/**********************************************************************
 * OtaManager.cpp
 *
 * 网络管理器实现（原生 ESP-IDF）：WiFi / NTP / HTTP 服务 / SD 文件管理 /
 * 固件 OTA 上传。见 OtaManager.h 顶部接口说明。
 *********************************************************************/

#include "OtaManager.h"
#include "RamCapture.h"

#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>

#include "OtaConfig.h"
#include "SdCard.h"
#include "Sys.h"
#include "esp_app_format.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_sntp.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

// ── 浏览器文件管理页面（SD 卡浏览/下载/上传/删除）─────────────────────
// 网页控制台（日志 + 命令）已于 2026-09-19 删除：控制只走 USB 串口和 HC-12。
static const char CONSOLE_HTML[] = R"HTML(<!DOCTYPE html><html lang="zh-CN"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Squid Robot 文件管理</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/jszip/3.10.1/jszip.min.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0;font-family:-apple-system,'Segoe UI',Roboto,sans-serif}
body{background:#1a1a1a;color:#ddd;height:100vh;display:flex;flex-direction:column;font-size:13px;overflow:hidden}
.title{padding:6px 12px;background:#0d2a4f;color:#fff;font-weight:600;font-size:13px;border-bottom:1px solid #1a4070}
.bar{padding:6px 8px;background:#252525;border-bottom:1px solid #333;display:flex;align-items:center;gap:6px;flex-wrap:wrap}
.bar button{background:#2d4f7f;color:#fff;border:0;padding:5px 10px;border-radius:3px;cursor:pointer;font-size:12px}
.bar button:hover:not(:disabled){background:#3a629a}
.bar button:disabled{background:#333;color:#666;cursor:default}
.path{flex:1;color:#8fcfff;font-family:Consolas,monospace;padding:0 6px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;min-width:80px}
#files{flex:1;overflow:auto;background:#202020;min-height:120px}
table{width:100%;border-collapse:collapse}
th{position:sticky;top:0;background:#2a2a2a;text-align:left;padding:6px 10px;font-weight:600;color:#aaa;border-bottom:1px solid #333;font-size:11px}
th.r{text-align:right}
th.cb,td.cb{width:32px;padding:4px 4px 4px 12px;text-align:center}
th.cb input,td.cb input{cursor:pointer;width:14px;height:14px;accent-color:#3a629a}
td{padding:4px 10px;border-bottom:1px solid #2a2a2a;cursor:pointer;user-select:none}
tr:hover td{background:#2a2a2a}
tr.sel td{background:#2d4f7f55}
td.r{text-align:right;color:#888;font-family:Consolas,monospace}
.dir{color:#8fcfff}
.file{color:#ddd}
.fs{padding:4px 12px;color:#888;font-size:11px;background:#1f1f1f;border-bottom:1px solid #333;border-top:1px solid #333}
input[type=file]{display:none}
</style></head>
<body>
<div class="title">🦑 Squid Robot 文件管理</div>
<div class="bar">
  <button id="up">↑ 上级</button>
  <span class="path" id="path">/</span>
  <button id="refresh">⟳ 刷新</button>
  <button id="upload">⬆ 上传</button>
  <button id="download" disabled>⬇ 下载</button>
  <button id="delete" disabled>✕ 删除</button>
  <input type="file" id="filepick" multiple>
</div>
<div id="files"><table>
  <thead><tr>
    <th class="cb"><input type="checkbox" id="selAll" title="全选/全不选"></th>
    <th>名称</th>
    <th class="r">大小</th>
  </tr></thead>
  <tbody id="tb"></tbody>
</table></div>
<div class="fs" id="fs">—</div>
<script>
const $=id=>document.getElementById(id);
const tb=$('tb'),path=$('path'),fs=$('fs');
let cur='/';
let curItems=[];
const sel=new Set();
function fmt(n){
  if(n<1024) return n+' B';
  if(n<1048576) return (n/1024).toFixed(1)+' KB';
  if(n<1073741824) return (n/1048576).toFixed(2)+' MB';
  return (n/1073741824).toFixed(2)+' GB';
}
function join(base,name){return (base==='/'?'':base.replace(/\/$/,''))+'/'+name;}
function refreshButtons(){const has=sel.size>0;$('download').disabled=!has;$('delete').disabled=!has;}
function refreshSelAll(){
  const cbs=tb.querySelectorAll('input.cb');const all=$('selAll');
  if(cbs.length===0){all.checked=false;all.indeterminate=false;return;}
  let n=0;cbs.forEach(c=>{if(c.checked)n++;});
  all.checked=(n===cbs.length);all.indeterminate=(n>0&&n<cbs.length);
}
function toggleRow(tr,checked){
  const name=tr.dataset.name;const cb=tr.querySelector('input.cb');
  if(cb) cb.checked=checked;
  if(checked){sel.add(name);tr.classList.add('sel');}else{sel.delete(name);tr.classList.remove('sel');}
  refreshButtons();refreshSelAll();
}
async function load(p){
  fs.textContent='读取 '+p+' ...';
  try{
    const r=await fetch('/files?path='+encodeURIComponent(p));
    if(!r.ok) throw new Error('HTTP '+r.status);
    const j=await r.json();
    cur=j.path||p;path.textContent=cur;sel.clear();
    $('selAll').checked=false;$('selAll').indeterminate=false;$('up').disabled=(cur==='/');
    const items=(j.items||[]).slice().sort((a,b)=>{
      if(a.dir!==b.dir) return a.dir?-1:1;
      return (a.name||'').localeCompare(b.name||'');
    }).filter(it=>!!it.name);
    curItems=items;tb.innerHTML='';
    items.forEach(it=>{
      const tr=document.createElement('tr');
      tr.dataset.name=it.name;tr.dataset.dir=it.dir?'1':'';
      const tdC=document.createElement('td');tdC.className='cb';
      const cb=document.createElement('input');cb.type='checkbox';cb.className='cb';
      cb.onclick=e=>{e.stopPropagation();toggleRow(tr,cb.checked);};
      tdC.appendChild(cb);
      const tdN=document.createElement('td');tdN.className=it.dir?'dir':'file';
      tdN.textContent=(it.dir?'📁 ':'📄 ')+it.name;
      const tdS=document.createElement('td');tdS.className='r';
      tdS.textContent=it.dir?'':fmt(it.size||0);
      tr.appendChild(tdC);tr.appendChild(tdN);tr.appendChild(tdS);
      tr.onclick=e=>{if(e.target===cb||e.target.closest('td.cb'))return;if(it.dir)load(join(cur,it.name));};
      tr.ondblclick=e=>{if(e.target===cb||e.target.closest('td.cb'))return;if(it.dir)load(join(cur,it.name));else doDownload(it.name);};
      tb.appendChild(tr);
    });
    refreshButtons();refreshSelAll();fs.textContent='就绪 — '+items.length+' 项';
  }catch(e){fs.textContent='读取失败: '+e.message;}
}
$('selAll').onchange=e=>{const check=e.target.checked;tb.querySelectorAll('tr').forEach(tr=>toggleRow(tr,check));};
function doDownload(name){
  const a=document.createElement('a');a.href='/file?path='+encodeURIComponent(join(cur,name));a.download=name;
  document.body.appendChild(a);a.click();a.remove();
}
async function collectIntoZip(zip,parentDir,item,prefix){
  const fullPath=join(parentDir,item.name);const entryName=prefix+item.name;
  if(item.dir){
    const r=await fetch('/files?path='+encodeURIComponent(fullPath));
    if(!r.ok) throw new Error('list '+fullPath+' failed');
    const j=await r.json();zip.folder(entryName);
    for(const child of (j.items||[])){if(!child.name) continue;await collectIntoZip(zip,fullPath,child,entryName+'/');}
  }else{
    const r=await fetch('/file?path='+encodeURIComponent(fullPath));
    if(!r.ok) throw new Error('download '+fullPath+' failed');
    const buf=await r.arrayBuffer();zip.file(entryName,buf);
  }
}
$('up').onclick=()=>{if(cur==='/') return;const idx=cur.lastIndexOf('/');load(idx<=0?'/':cur.substring(0,idx));};
$('refresh').onclick=()=>load(cur);
$('download').onclick=async()=>{
  if(sel.size===0) return;
  const items=curItems.filter(it=>sel.has(it.name));
  if(items.length===1&&!items[0].dir){doDownload(items[0].name);return;}
  if(typeof JSZip==='undefined'){fs.textContent='打包失败：JSZip 未加载（需互联网访问 cdnjs）';return;}
  fs.textContent='正在打包 '+items.length+' 项...';
  try{
    const zip=new JSZip();let i=0;
    for(const it of items){await collectIntoZip(zip,cur,it,'');i++;fs.textContent='正在打包... ('+i+'/'+items.length+')';}
    fs.textContent='生成 ZIP...';
    const blob=await zip.generateAsync({type:'blob'},m=>{fs.textContent='生成 ZIP... '+m.percent.toFixed(0)+'%';});
    const baseName=(items.length===1?items[0].name:('squid-'+cur.replace(/[\/\s]+/g,'_').replace(/^_+|_+$/g,'')||'root'));
    const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download=baseName+'.zip';
    document.body.appendChild(a);a.click();setTimeout(()=>{URL.revokeObjectURL(a.href);a.remove();},200);
    fs.textContent='✓ 下载完成: '+a.download+' ('+fmt(blob.size)+')';
  }catch(e){fs.textContent='打包失败: '+e.message;}
};
$('delete').onclick=async()=>{
  if(sel.size===0) return;
  const names=[...sel];
  const preview=names.length<=8?names.join('\n'):(names.slice(0,8).join('\n')+'\n...（共 '+names.length+' 项）');
  if(!confirm('确定删除以下 '+names.length+' 项？\n\n'+preview+'\n\n此操作不可恢复。')) return;
  fs.textContent='删除 '+names.length+' 项...';let ok=0,fail=0;
  for(const name of names){
    try{const r=await fetch('/file?path='+encodeURIComponent(join(cur,name)),{method:'DELETE'});if(r.ok) ok++; else fail++;}catch{fail++;}
    fs.textContent='删除中... ('+(ok+fail)+'/'+names.length+')';
  }
  fs.textContent='✓ 已删除 '+ok+(fail>0?'，失败 '+fail:'');load(cur);
};
$('upload').onclick=()=>$('filepick').click();
$('filepick').onchange=async(ev)=>{
  const files=[...ev.target.files];if(files.length===0) return;let ok=0,fail=0;
  for(let i=0;i<files.length;i++){
    const f=files[i];fs.textContent='上传 ('+(i+1)+'/'+files.length+') '+f.name+' ('+fmt(f.size)+')...';
    try{
      const fd=new FormData();fd.append('file',f,f.name);
      const r=await fetch('/upload?path='+encodeURIComponent(cur),{method:'POST',body:fd});
      if(!r.ok) throw new Error('HTTP '+r.status);ok++;
    }catch(e){fail++;}
  }
  fs.textContent='✓ 上传完成 '+ok+(fail>0?'，失败 '+fail:'');ev.target.value='';load(cur);
};
load('/');
</script>
</body></html>)HTML";

// ────────────────────────────────────────────────────────────────────
namespace {
constexpr const char* TAG_HOST = "ota";
EventGroupHandle_t s_wifiEvents = nullptr;
constexpr int WIFI_CONNECTED_BIT = BIT0;
constexpr int WIFI_FAIL_BIT = BIT1;
esp_netif_t* s_staNetif = nullptr;
esp_netif_t* s_apNetif = nullptr;
bool s_netStackReady = false;

TaskHandle_t   s_dnsTask = nullptr;
volatile bool  s_dnsRun = false;
constexpr const char* PROV_AP_SSID = "SquidRobot-Setup";

bool hasValue(const char* v) { return v && v[0] != '\0'; }

// 极简 DNS：把所有查询都应答成 192.168.4.1，实现 Captive Portal。
void dnsCaptiveTask(void*) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { vTaskDelete(nullptr); return; }
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(53);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(sock);
        vTaskDelete(nullptr);
        return;
    }
    struct timeval tv = {1, 0};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint8_t buf[512];
    while (s_dnsRun) {
        struct sockaddr_in from = {};
        socklen_t fromLen = sizeof(from);
        int n = recvfrom(sock, buf, sizeof(buf), 0,
                         reinterpret_cast<struct sockaddr*>(&from), &fromLen);
        if (n < 12) continue;

        // 构造应答：复用请求头，置 QR/AA，Answer=1，指向 192.168.4.1。
        buf[2] |= 0x80;   // QR = response
        buf[3] = 0x80;    // RA
        buf[7] = 1;       // ANCOUNT = 1（保留 QDCOUNT=buf[4..5]）
        if (n + 16 > static_cast<int>(sizeof(buf))) continue;
        uint8_t* p = buf + n;
        *p++ = 0xC0; *p++ = 0x0C;                 // 指针指向问题里的域名
        *p++ = 0x00; *p++ = 0x01;                 // TYPE A
        *p++ = 0x00; *p++ = 0x01;                 // CLASS IN
        *p++ = 0x00; *p++ = 0x00; *p++ = 0x00; *p++ = 0x3C;  // TTL 60
        *p++ = 0x00; *p++ = 0x04;                 // RDLENGTH 4
        *p++ = 192; *p++ = 168; *p++ = 4; *p++ = 1;
        sendto(sock, buf, n + 16, 0,
               reinterpret_cast<struct sockaddr*>(&from), fromLen);
    }
    close(sock);
    vTaskDelete(nullptr);
}

void wifiEventHandler(void*, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_wifiEvents) {
            xEventGroupClearBits(s_wifiEvents, WIFI_CONNECTED_BIT);
            xEventGroupSetBits(s_wifiEvents, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        if (s_wifiEvents) {
            xEventGroupClearBits(s_wifiEvents, WIFI_FAIL_BIT);
            xEventGroupSetBits(s_wifiEvents, WIFI_CONNECTED_BIT);
        }
    }
}

// 一次性初始化 TCP/IP 协议栈 + 事件循环 + WiFi 驱动。
bool netStackInitOnce() {
    if (s_netStackReady) return true;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    if (esp_netif_init() != ESP_OK) return false;
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return false;

    s_staNetif = esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&cfg) != ESP_OK) return false;

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandler, nullptr, nullptr);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifiEventHandler, nullptr, nullptr);

    s_wifiEvents = xEventGroupCreate();
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    s_netStackReady = true;
    return true;
}
}  // namespace

// ── OtaManager ───────────────────────────────────────────────────────
OtaManager::OtaManager()
    : _ready(false),
      _consoleActive(false),
      _ntpSynced(false),
      _reconnectEnabled(false),
      _nextReconnectMs(0),
      _ntpEpoch(0),
      _ntpSyncMs(0),
      _apActive(false),
      _provisionPending(false),
      _httpd(nullptr) {}

void OtaManager::begin(bool underwaterMode) {
    _ready = false;
    _consoleActive = false;
    _reconnectEnabled = false;

    if (underwaterMode) {
        g_dbg->println("OTA: underwater mode detected, WiFi disabled.");
        return;
    }

    if (!netStackInitOnce()) {
        g_dbg->println("WiFi: net stack init failed.");
        return;
    }

    // ① 配置里指定了 WiFi 就先连它。连上后 esp_wifi 会把凭据写进 NVS，
    //    所以下次即使固件里的配置被清空，NVS 里也还留着这一个。
    if (hasValue(WIFI_FALLBACK_SSID)) {
        g_dbg->printf("WiFi: 优先尝试指定网络 '%s'\r\n", WIFI_FALLBACK_SSID);
        if (connectSta(WIFI_FALLBACK_SSID, WIFI_FALLBACK_PASS, OTA_CONNECT_TIMEOUT_MS)) {
            finishStaSetup();
            return;
        }
        g_dbg->println("WiFi: 指定网络连不上，退回 NVS 里存的网络。");
    }

    // ② 退回 NVS 里存的上一个网络 —— 指定网络配错了也还有路可走。
    if (!beginNvsMode()) {
        // 都连不上：开配网热点（不阻塞，主循环继续跑）。
        g_dbg->println("WiFi: 未连上外部网络，开启配网热点 SquidRobot-Setup。");
        g_dbg->println("      （配网期间也可直接烧录：http://192.168.4.1/ota）");
        beginProvisioningPortal();
        return;
    }

    finishStaSetup();
}

void OtaManager::finishStaSetup() {
    esp_netif_set_hostname(s_staNetif, OTA_HOSTNAME);
    beginAlwaysOnAp();   // 外部 WiFi 连上了也留着自己的热点，多一条烧录入口
    syncNtp();
    _ready = true;
    _reconnectEnabled = true;
    _nextReconnectMs = millis() + 5000U;
}

void OtaManager::handle() {
    // Retry from the low-priority main loop, not the Wi-Fi event callback or
    // motion task. No waits, Wi-Fi resets, NVS writes, or HTTP-server teardown.
    // Keep RAM captures and the always-on recovery AP intact while underwater.
    if (_reconnectEnabled && !_apActive && !_provisionPending && s_wifiEvents) {
        const EventBits_t bits = xEventGroupGetBits(s_wifiEvents);
        const uint32_t now = millis();
        if (bits & WIFI_CONNECTED_BIT) {
            _ready = true;
            _nextReconnectMs = now + 5000U;
        } else {
            _ready = false;
            if ((bits & WIFI_FAIL_BIT) &&
                static_cast<int32_t>(now - _nextReconnectMs) >= 0) {
                // A failure event schedules the next attempt; do not interrupt
                // an association already in progress just because DHCP is slow.
                xEventGroupClearBits(s_wifiEvents, WIFI_FAIL_BIT);
                const esp_err_t result = esp_wifi_connect();
                if (result != ESP_OK) xEventGroupSetBits(s_wifiEvents, WIFI_FAIL_BIT);
                _nextReconnectMs = now + 5000U;
            }
        }
    }
    // esp_http_server 在自己的任务里跑；这里只处理配网页面提交的待连接凭据，
    // 在主循环（而非 httpd 任务）里完成连接和热点拆除，避免自拆服务器。
    if (_provisionPending) {
        _provisionPending = false;
        g_dbg->print("[Portal] 尝试连接: ");
        g_dbg->println(_pendingSsid);

        wifi_config_t conf = {};
        strlcpy(reinterpret_cast<char*>(conf.sta.ssid), _pendingSsid.c_str(), sizeof(conf.sta.ssid));
        if (!_pendingPass.empty()) {
            strlcpy(reinterpret_cast<char*>(conf.sta.password), _pendingPass.c_str(), sizeof(conf.sta.password));
        }
        esp_wifi_set_config(WIFI_IF_STA, &conf);   // storage=FLASH，成功即持久化
        xEventGroupClearBits(s_wifiEvents, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
        esp_wifi_connect();

        EventBits_t bits = xEventGroupWaitBits(
            s_wifiEvents, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE,
            pdMS_TO_TICKS(OTA_CONNECT_TIMEOUT_MS));

        if (bits & WIFI_CONNECTED_BIT) {
            stopProvisioningPortal();
            finishStaSetup();
            char ip[16];
            getLocalIpStr(ip, sizeof(ip));
            g_dbg->print("[Portal] 配网成功，IP: ");
            g_dbg->println(ip);
        } else {
            esp_wifi_disconnect();
            g_dbg->println("[Portal] 连接失败，热点继续开放，请在配网页重试。");
        }
    }
}

void OtaManager::queueProvisionCreds(const char* ssid, const char* pass) {
    _pendingSsid = ssid ? ssid : "";
    _pendingPass = pass ? pass : "";
    _provisionPending = true;
}

bool OtaManager::isReady() const { return _ready; }

void OtaManager::getLocalIpStr(char* buf, size_t len) const {
    esp_netif_ip_info_t ip = {};
    if (s_staNetif && esp_netif_get_ip_info(s_staNetif, &ip) == ESP_OK) {
        snprintf(buf, len, IPSTR, IP2STR(&ip.ip));
    } else {
        snprintf(buf, len, "0.0.0.0");
    }
}

// ── WiFi ─────────────────────────────────────────────────────────────
bool OtaManager::beginNvsMode() {
    esp_wifi_set_mode(WIFI_MODE_STA);
    if (esp_wifi_start() != ESP_OK) return false;

    // 用 NVS 里保存的 SSID/密码连接（esp_wifi 以 FLASH 存储时自动加载）。
    xEventGroupClearBits(s_wifiEvents, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    esp_wifi_connect();

    g_dbg->print("WiFi: trying NVS saved credentials");
    EventBits_t bits = xEventGroupWaitBits(
        s_wifiEvents, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE,
        pdMS_TO_TICKS(10000));
    g_dbg->println();

    if (bits & WIFI_CONNECTED_BIT) {
        char ip[16];
        getLocalIpStr(ip, sizeof(ip));
        g_dbg->print("WiFi connected (NVS), IP: ");
        g_dbg->println(ip);
        return true;
    }

    esp_wifi_disconnect();
    return false;
}

bool OtaManager::connectSta(const char* ssid, const char* pass, uint32_t timeoutMs) {
    wifi_config_t conf = {};
    strlcpy(reinterpret_cast<char*>(conf.sta.ssid), ssid, sizeof(conf.sta.ssid));
    if (pass && pass[0]) {
        strlcpy(reinterpret_cast<char*>(conf.sta.password), pass, sizeof(conf.sta.password));
    }
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &conf);   // storage=FLASH，自动持久化到 NVS
    esp_wifi_start();

    xEventGroupClearBits(s_wifiEvents, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    esp_wifi_connect();

    g_dbg->print("WiFi: connecting");
    EventBits_t bits = xEventGroupWaitBits(
        s_wifiEvents, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE,
        pdMS_TO_TICKS(timeoutMs));
    g_dbg->println();

    if (bits & WIFI_CONNECTED_BIT) {
        char ip[16];
        getLocalIpStr(ip, sizeof(ip));
        g_dbg->print("WiFi connected, IP: ");
        g_dbg->println(ip);
        return true;
    }

    esp_wifi_disconnect();
    g_dbg->println("WiFi connect timeout.");
    return false;
}

// ── NTP ──────────────────────────────────────────────────────────────
bool OtaManager::syncNtp() {
    // UTC+9（韩国标准时间 KST）。
    setenv("TZ", "KST-9", 1);
    tzset();

    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.google.com");
    esp_sntp_setservername(2, "kr.pool.ntp.org");
    esp_sntp_init();

    // 等待时间被设置（最多 5s）。
    const uint32_t start = millis();
    struct tm timeinfo = {};
    time_t now = 0;
    while (millis() - start < 5000) {
        time(&now);
        localtime_r(&now, &timeinfo);
        if (timeinfo.tm_year > (2020 - 1900)) break;
        delay(200);
    }

    if (timeinfo.tm_year <= (2020 - 1900)) {
        g_dbg->println("NTP: sync failed.");
        return false;
    }

    _ntpEpoch = now;
    _ntpSyncMs = millis();
    _ntpSynced = true;

    char buf[24];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    g_dbg->print("NTP synced: ");
    g_dbg->println(buf);
    return true;
}

bool OtaManager::hasTime() const { return _ntpSynced; }

void OtaManager::getSessionFolderName(char* buf, size_t len) const {
    if (!_ntpSynced) {
        strncpy(buf, "no_time", len);
        buf[len - 1] = '\0';
        return;
    }
    const time_t now = _ntpEpoch + static_cast<time_t>((millis() - _ntpSyncMs) / 1000UL);
    struct tm t;
    localtime_r(&now, &t);
    strftime(buf, len, "%Y-%m-%d_%H-%M-%S", &t);
}


// ── HTTP 工具 ────────────────────────────────────────────────────────
namespace {
// URL 解码（%XX 和 '+'）。
void urlDecode(const char* src, char* dst, size_t dstLen) {
    size_t di = 0;
    for (size_t si = 0; src[si] && di + 1 < dstLen; ++si) {
        if (src[si] == '%' && src[si + 1] && src[si + 2]) {
            auto hex = [](char c) -> int {
                if (c >= '0' && c <= '9') return c - '0';
                if (c >= 'a' && c <= 'f') return c - 'a' + 10;
                if (c >= 'A' && c <= 'F') return c - 'A' + 10;
                return 0;
            };
            dst[di++] = static_cast<char>((hex(src[si + 1]) << 4) | hex(src[si + 2]));
            si += 2;
        } else if (src[si] == '+') {
            dst[di++] = ' ';
        } else {
            dst[di++] = src[si];
        }
    }
    dst[di] = '\0';
}

// 取 URL query 里的参数并做 URL 解码；无则返回 false。
bool getQueryParam(httpd_req_t* req, const char* key, char* out, size_t outLen) {
    size_t qlen = httpd_req_get_url_query_len(req) + 1;
    if (qlen <= 1) return false;
    char* q = static_cast<char*>(malloc(qlen));
    if (!q) return false;
    bool ok = false;
    if (httpd_req_get_url_query_str(req, q, qlen) == ESP_OK) {
        char raw[256];
        if (httpd_query_key_value(q, key, raw, sizeof(raw)) == ESP_OK) {
            urlDecode(raw, out, outLen);
            ok = true;
        }
    }
    free(q);
    return ok;
}

// 递归删除文件或目录（逻辑路径）。
bool removeRecursiveLogical(const char* logical) {
    char full[128];
    sdFsPath(full, sizeof(full), logical);

    struct stat st;
    if (stat(full, &st) != 0) return false;

    if (!S_ISDIR(st.st_mode)) {
        return unlink(full) == 0;
    }

    DIR* dir = opendir(full);
    if (!dir) return false;
    struct dirent* ent;
    bool ok = true;
    while ((ent = readdir(dir)) != nullptr) {
        if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) continue;
        char childLogical[128];
        snprintf(childLogical, sizeof(childLogical), "%s/%s", logical, ent->d_name);
        if (!removeRecursiveLogical(childLogical)) { ok = false; break; }
    }
    closedir(dir);
    if (!ok) return false;
    return rmdir(full) == 0;
}
}  // namespace

// ── HTTP 处理器 ──────────────────────────────────────────────────────
static esp_err_t h_consoleHtml(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, CONSOLE_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t h_files(httpd_req_t* req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    char logical[128] = "/";
    getQueryParam(req, "path", logical, sizeof(logical));
    if (logical[0] == '\0') strcpy(logical, "/");

    char full[160];
    sdFsPath(full, sizeof(full), logical);

    DIR* dir = opendir(full);
    if (!dir) {
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "{\"error\":\"not a directory\"}", HTTPD_RESP_USE_STRLEN);
    }

    std::string json = "{\"path\":\"";
    json += logical;
    json += "\",\"items\":[";
    bool first = true;
    struct dirent* ent;
    while ((ent = readdir(dir)) != nullptr) {
        if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) continue;
        char childFull[220];
        snprintf(childFull, sizeof(childFull), "%s/%s", full, ent->d_name);
        struct stat st;
        bool isDir = false;
        long size = 0;
        if (stat(childFull, &st) == 0) {
            isDir = S_ISDIR(st.st_mode);
            size = static_cast<long>(st.st_size);
        }
        if (!first) json += ",";
        first = false;
        json += "{\"name\":\"";
        json += ent->d_name;
        json += "\",\"dir\":";
        json += isDir ? "true" : "false";
        if (!isDir) {
            json += ",\"size\":";
            json += std::to_string(size);
        }
        json += "}";
    }
    closedir(dir);
    json += "]}";

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json.data(), json.size());
}

static esp_err_t h_fileGet(httpd_req_t* req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    char logical[128];
    if (!getQueryParam(req, "path", logical, sizeof(logical))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing path");
        return ESP_FAIL;
    }
    char full[160];
    sdFsPath(full, sizeof(full), logical);

    struct stat st;
    if (stat(full, &st) != 0 || S_ISDIR(st.st_mode)) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "not found");
        return ESP_FAIL;
    }
    FILE* f = fopen(full, "rb");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "not found");
        return ESP_FAIL;
    }

    const char* ct = "application/octet-stream";
    size_t n = strlen(logical);
    auto ends = [&](const char* ext) {
        size_t e = strlen(ext);
        return n >= e && strcmp(logical + n - e, ext) == 0;
    };
    if (ends(".csv")) ct = "text/csv";
    else if (ends(".log") || ends(".txt")) ct = "text/plain";
    else if (ends(".json")) ct = "application/json";
    httpd_resp_set_type(req, ct);

    const char* leaf = strrchr(logical, '/');
    leaf = leaf ? leaf + 1 : logical;
    char disp[160];
    snprintf(disp, sizeof(disp), "attachment; filename=\"%s\"", leaf);
    httpd_resp_set_hdr(req, "Content-Disposition", disp);

    char buf[2048];
    size_t rd;
    esp_err_t err = ESP_OK;
    while ((rd = fread(buf, 1, sizeof(buf), f)) > 0) {
        if (httpd_resp_send_chunk(req, buf, rd) != ESP_OK) { err = ESP_FAIL; break; }
    }
    fclose(f);
    httpd_resp_send_chunk(req, nullptr, 0);
    return err;
}

static esp_err_t h_fileDelete(httpd_req_t* req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    char logical[128];
    if (!getQueryParam(req, "path", logical, sizeof(logical))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing path");
        return ESP_FAIL;
    }
    if (strcmp(logical, "/") == 0 || logical[0] == '\0') {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "refuse to delete root");
        return ESP_FAIL;
    }
    char full[160];
    sdFsPath(full, sizeof(full), logical);
    struct stat st;
    if (stat(full, &st) != 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "not found");
        return ESP_FAIL;
    }
    const bool ok = removeRecursiveLogical(logical);
    if (ok) return httpd_resp_sendstr(req, "OK");
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "delete failed");
}

// POST /upload?path=/dir  —— multipart/form-data 流式上传。
static esp_err_t h_upload(httpd_req_t* req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char dir[128] = "/";
    getQueryParam(req, "path", dir, sizeof(dir));
    if (dir[0] == '\0') strcpy(dir, "/");

    // 取 boundary。
    char ctype[128] = {0};
    if (httpd_req_get_hdr_value_str(req, "Content-Type", ctype, sizeof(ctype)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no content-type");
        return ESP_FAIL;
    }
    const char* bpos = strstr(ctype, "boundary=");
    if (!bpos) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no boundary");
        return ESP_FAIL;
    }
    std::string closeDelim = "\r\n--";
    closeDelim += (bpos + 9);

    int remaining = req->content_len;
    char rbuf[1024];

    // Phase 1：读到 part 头结束（\r\n\r\n），提取 filename。
    std::string head;
    std::string leftover;   // 头之后已读到的数据
    bool headerDone = false;
    while (!headerDone && remaining > 0) {
        int r = httpd_req_recv(req, rbuf, sizeof(rbuf) < (size_t)remaining ? sizeof(rbuf) : remaining);
        if (r <= 0) break;
        remaining -= r;
        head.append(rbuf, r);
        size_t p = head.find("\r\n\r\n");
        if (p != std::string::npos) {
            leftover = head.substr(p + 4);
            head.resize(p);
            headerDone = true;
        }
        if (head.size() > 4096) break;   // 防御：头异常过长
    }

    std::string leaf;
    size_t fpos = head.find("filename=\"");
    if (fpos != std::string::npos) {
        fpos += 10;
        size_t fend = head.find('"', fpos);
        if (fend != std::string::npos) leaf = head.substr(fpos, fend - fpos);
    }
    size_t slash = leaf.find_last_of('/');
    if (slash != std::string::npos) leaf = leaf.substr(slash + 1);
    size_t bslash = leaf.find_last_of('\\');
    if (bslash != std::string::npos) leaf = leaf.substr(bslash + 1);

    if (leaf.empty()) {
        // 排空剩余 body。
        while (remaining > 0) {
            int r = httpd_req_recv(req, rbuf, sizeof(rbuf) < (size_t)remaining ? sizeof(rbuf) : remaining);
            if (r <= 0) break;
            remaining -= r;
        }
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty filename");
        return ESP_FAIL;
    }

    std::string logical = dir;
    if (logical.empty() || logical.back() != '/') logical += '/';
    logical += leaf;
    char full[192];
    sdFsPath(full, sizeof(full), logical.c_str());

    FILE* f = fopen(full, "wb");

    // Phase 2：写数据直到遇到 closeDelim。用 pending 缓冲跨块检测边界。
    std::string pending = leftover;
    const size_t keep = closeDelim.size();   // 需保留可能是边界起始的尾部
    bool done = false;
    while (!done) {
        size_t dp = pending.find(closeDelim);
        if (dp != std::string::npos) {
            if (f && dp > 0) fwrite(pending.data(), 1, dp, f);
            done = true;
            break;
        }
        if (pending.size() > keep) {
            size_t writeLen = pending.size() - keep;
            if (f) fwrite(pending.data(), 1, writeLen, f);
            pending.erase(0, writeLen);
        }
        if (remaining <= 0) {
            // 没有更多数据：把剩下的（去掉可能的结尾）写完。
            if (f && !pending.empty()) fwrite(pending.data(), 1, pending.size(), f);
            break;
        }
        int r = httpd_req_recv(req, rbuf, sizeof(rbuf) < (size_t)remaining ? sizeof(rbuf) : remaining);
        if (r <= 0) break;
        remaining -= r;
        pending.append(rbuf, r);
    }

    if (f) fclose(f);
    return httpd_resp_sendstr(req, f ? "OK" : "open failed");
}

// POST /ota  —— 固件上传（application/octet-stream），走 esp_ota。
static esp_err_t h_ota(httpd_req_t* req) {
    OtaManager* self = static_cast<OtaManager*>(req->user_ctx);
    (void)self;

    const esp_partition_t* target = esp_ota_get_next_update_partition(nullptr);
    if (!target) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no ota partition");
        return ESP_FAIL;
    }

    esp_ota_handle_t handle = 0;
    if (esp_ota_begin(target, OTA_SIZE_UNKNOWN, &handle) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota begin failed");
        return ESP_FAIL;
    }

    g_dbg->println("\r\nOTA start (HTTP upload)");
    int remaining = req->content_len;
    char buf[2048];
    bool fail = false;
    while (remaining > 0) {
        int r = httpd_req_recv(req, buf, sizeof(buf) < (size_t)remaining ? sizeof(buf) : remaining);
        if (r <= 0) { fail = true; break; }
        if (esp_ota_write(handle, buf, r) != ESP_OK) { fail = true; break; }
        remaining -= r;
    }

    if (fail || esp_ota_end(handle) != ESP_OK) {
        esp_ota_abort(handle);
        g_dbg->println("OTA failed");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota write failed");
        return ESP_FAIL;
    }

    if (esp_ota_set_boot_partition(target) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "set boot failed");
        return ESP_FAIL;
    }

    g_dbg->println("OTA success, rebooting...");
    httpd_resp_sendstr(req, "OK, rebooting");
    delay(500);
    esp_restart();
    return ESP_OK;
}

// ── 配网门户 ────────────────────────────────────────────────────────
static const char PORTAL_HTML[] = R"HTML(<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Squid Robot WiFi 配网</title>
<style>
body{font-family:sans-serif;background:#1a1a2e;color:#eee;display:flex;justify-content:center;padding:32px 16px;margin:0}
.card{background:#16213e;border-radius:12px;padding:28px;width:100%;max-width:380px}
h2{margin:0 0 8px;color:#00d4ff;text-align:center;font-size:20px}
.sub{text-align:center;color:#888;font-size:13px;margin-bottom:24px}
label{display:block;margin-bottom:6px;font-size:13px;color:#aaa}
input{width:100%;box-sizing:border-box;padding:10px;border-radius:6px;border:1px solid #0f3460;background:#0f3460;color:#fff;font-size:15px;margin-bottom:16px}
button{width:100%;padding:12px;border-radius:6px;border:none;background:#00d4ff;color:#1a1a2e;font-size:16px;font-weight:bold;cursor:pointer}
.net{padding:10px 14px;background:#0f3460;border-radius:6px;margin-bottom:8px;cursor:pointer;font-size:14px;display:flex;justify-content:space-between;align-items:center}
.net:hover{background:#1a4a8a}
.rssi{color:#888;font-size:12px}
hr{border:none;border-top:1px solid #0f3460;margin:20px 0}
</style></head>
<body><div class="card">
<h2>&#x1F991; Squid Robot</h2>
<p class="sub">WiFi 配网 — 连接后可用网页控制台 / OTA</p>
<form method="POST" action="/save">
<label>WiFi 名称 (SSID)</label>
<input type="text" name="ssid" id="ssid" placeholder="输入或点击下方网络" required>
<label>密码</label>
<input type="password" name="pass" placeholder="无密码留空">
<button type="submit">连接并保存</button>
</form>
<hr>
<div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:10px">
<p style="color:#aaa;font-size:13px;margin:0">附近网络：</p>
<button type="button" id="scanBtn" onclick="doScan()" style="width:auto;padding:6px 14px;font-size:13px;background:#0f3460;color:#00d4ff;border:1px solid #00d4ff;border-radius:6px;cursor:pointer">重新扫描</button>
</div>
<div id="netList"><p style="color:#888;font-size:13px">扫描中...</p></div>
</div>
<script>
function fill(s){document.getElementById('ssid').value=s}
function doScan(){
 var b=document.getElementById('scanBtn'),l=document.getElementById('netList');
 b.disabled=true;b.textContent='扫描中...';l.innerHTML='<p style="color:#888;font-size:13px">扫描中，请稍候...</p>';
 fetch('/scan').then(r=>r.text()).then(h=>{l.innerHTML=h;b.disabled=false;b.textContent='重新扫描';}).catch(()=>{b.disabled=false;b.textContent='重新扫描';});
}
doScan();
</script>
</body></html>)HTML";

static const char PORTAL_SUCCESS_HTML[] = R"HTML(<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>正在连接...</title>
<style>body{font-family:sans-serif;background:#1a1a2e;color:#eee;display:flex;justify-content:center;padding:60px 16px}
.card{background:#16213e;border-radius:12px;padding:32px;text-align:center;max-width:360px}h2{color:#00d4ff}
.hint{color:#888;font-size:13px;margin-top:16px}</style></head>
<body><div class="card"><h2>正在连接...</h2>
<p class="hint">机器人正在尝试连接该 WiFi。<br>连接成功后本热点会自动关闭，<br>请到路由器/串口查看机器人获得的 IP。<br>若失败，本热点会继续开放，请重试。</p>
</div></body></html>)HTML";

static std::string buildScanResults() {
    uint16_t n = 0;
    wifi_scan_config_t sc = {};
    if (esp_wifi_scan_start(&sc, true) != ESP_OK) {
        return "<p style='color:#888;font-size:13px'>扫描失败</p>";
    }
    esp_wifi_scan_get_ap_num(&n);
    if (n == 0) return "<p style='color:#888;font-size:13px'>未发现网络</p>";
    if (n > 20) n = 20;
    wifi_ap_record_t* recs = static_cast<wifi_ap_record_t*>(calloc(n, sizeof(wifi_ap_record_t)));
    if (!recs) return "<p style='color:#888;font-size:13px'>内存不足</p>";
    esp_wifi_scan_get_ap_records(&n, recs);

    std::string html;
    for (uint16_t i = 0; i < n; i++) {
        const char* ssid = reinterpret_cast<const char*>(recs[i].ssid);
        if (ssid[0] == '\0') continue;
        html += "<div class='net' onclick=\"fill('";
        html += ssid;
        html += "')\"><span>";
        html += ssid;
        html += "</span><span class='rssi'>";
        html += std::to_string(recs[i].rssi);
        html += "dBm</span></div>";
    }
    free(recs);
    if (html.empty()) html = "<p style='color:#888;font-size:13px'>未发现网络</p>";
    return html;
}

static esp_err_t h_portalRoot(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, PORTAL_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t h_portalScan(httpd_req_t* req) {
    std::string html = buildScanResults();
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, html.data(), html.size());
}

static esp_err_t h_portalSave(httpd_req_t* req) {
    OtaManager* self = static_cast<OtaManager*>(req->user_ctx);
    char body[512] = {0};
    int total = req->content_len;
    if (total <= 0 || total >= static_cast<int>(sizeof(body))) total = sizeof(body) - 1;
    int received = 0;
    while (received < total) {
        int r = httpd_req_recv(req, body + received, total - received);
        if (r <= 0) break;
        received += r;
    }
    body[received] = '\0';

    char rawSsid[128] = {0}, rawPass[128] = {0};
    char ssid[128] = {0}, pass[128] = {0};
    if (httpd_query_key_value(body, "ssid", rawSsid, sizeof(rawSsid)) == ESP_OK) {
        urlDecode(rawSsid, ssid, sizeof(ssid));
    }
    if (httpd_query_key_value(body, "pass", rawPass, sizeof(rawPass)) == ESP_OK) {
        urlDecode(rawPass, pass, sizeof(pass));
    }

    if (ssid[0] == '\0') {
        httpd_resp_set_type(req, "text/html");
        return httpd_resp_send(req,
            "<meta charset='UTF-8'><p style='color:red'>错误：SSID 不能为空</p>",
            HTTPD_RESP_USE_STRLEN);
    }

    self->queueProvisionCreds(ssid, pass);   // 真正的连接在主循环里做
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, PORTAL_SUCCESS_HTML, HTTPD_RESP_USE_STRLEN);
}

// 强制门户：任何未命中的 GET 都重定向到门户首页。
static esp_err_t h_portalRedirect(httpd_req_t* req, httpd_err_code_t) {
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_send(req, "", 0);
    return ESP_OK;
}

// 见 OtaConfig.h 的说明：这是焊死主控之后唯一不依赖外部网络的烧录入口。
void OtaManager::beginAlwaysOnAp() {
    if (!s_apNetif) s_apNetif = esp_netif_create_default_wifi_ap();

    esp_wifi_set_mode(WIFI_MODE_APSTA);
    wifi_config_t ap = {};
    strlcpy(reinterpret_cast<char*>(ap.ap.ssid), AP_SSID, sizeof(ap.ap.ssid));
    ap.ap.ssid_len = strlen(AP_SSID);
    strlcpy(reinterpret_cast<char*>(ap.ap.password), AP_PASSWORD, sizeof(ap.ap.password));
    ap.ap.channel = 1;
    ap.ap.max_connection = 4;
    // 有密码才用 WPA2；留空就退回开放热点（否则 esp_wifi 会直接拒绝配置）。
    ap.ap.authmode = (strlen(AP_PASSWORD) >= 8) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    esp_wifi_set_config(WIFI_IF_AP, &ap);
    esp_wifi_start();

    g_dbg->printf("WiFi: 机器人热点 '%s' 已开启 —— 连上它即可烧录 http://192.168.4.1/\r\n",
                  AP_SSID);
}

void OtaManager::beginProvisioningPortal() {
    if (_apActive) return;

    // AP 网卡（懒创建）。
    if (!s_apNetif) s_apNetif = esp_netif_create_default_wifi_ap();

    esp_wifi_set_mode(WIFI_MODE_APSTA);
    wifi_config_t ap = {};
    strlcpy(reinterpret_cast<char*>(ap.ap.ssid), PROV_AP_SSID, sizeof(ap.ap.ssid));
    ap.ap.ssid_len = strlen(PROV_AP_SSID);
    ap.ap.channel = 1;
    ap.ap.max_connection = 4;
    ap.ap.authmode = WIFI_AUTH_OPEN;   // 开放热点，方便手机直连
    esp_wifi_set_config(WIFI_IF_AP, &ap);
    esp_wifi_start();

    // HTTP 服务（若未起则起）+ 门户路由。
    if (!_httpd) {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.server_port = 80;
        config.max_uri_handlers = 12;
        config.stack_size = 8192;
        config.lru_purge_enable = true;
        if (httpd_start(&_httpd, &config) != ESP_OK) {
            g_dbg->println("Portal: HTTP 启动失败");
            return;
        }
    }
    auto reg = [&](const char* uri, httpd_method_t m, esp_err_t (*h)(httpd_req_t*)) {
        httpd_uri_t u = {};
        u.uri = uri; u.method = m; u.handler = h; u.user_ctx = this;
        httpd_register_uri_handler(_httpd, &u);
    };
    reg("/",     HTTP_GET,  h_portalRoot);
    reg("/save", HTTP_POST, h_portalSave);
    reg("/scan", HTTP_GET,  h_portalScan);
    // 配网期间也允许烧录：还没配过网的机器人同样得能救。
    reg("/ota",  HTTP_POST, h_ota);
    httpd_register_err_handler(_httpd, HTTPD_404_NOT_FOUND, h_portalRedirect);

    // DNS 强制门户。
    s_dnsRun = true;
    if (!s_dnsTask) {
        xTaskCreate(dnsCaptiveTask, "dns_captive", 3072, nullptr, 5, &s_dnsTask);
    }

    _apActive = true;
    g_dbg->println("Portal: 热点 'SquidRobot-Setup' 已开启，手机连上后浏览器打开任意网页即弹配网。");
    g_dbg->println("        或直接访问 http://192.168.4.1/");
}

void OtaManager::stopProvisioningPortal() {
    if (!_apActive) return;
    s_dnsRun = false;                 // DNS 任务自行退出
    s_dnsTask = nullptr;
    if (_httpd) {                     // 停掉门户 HTTP（之后 beginWebServer 会重开文件管理/OTA）
        httpd_stop(_httpd);
        _httpd = nullptr;
    }
    _apActive = false;
    beginAlwaysOnAp();                // 不关热点，只是从开放的配网热点切回常开热点
    g_dbg->println("Portal: 配网完成，已切回常开热点。");
}

// ── 路由注册与启动 ──────────────────────────────────────────────────
void OtaManager::registerRoutes() {
    auto reg = [&](const char* uri, httpd_method_t method, esp_err_t (*handler)(httpd_req_t*)) {
        httpd_uri_t u = {};
        u.uri = uri;
        u.method = method;
        u.handler = handler;
        u.user_ctx = this;
        httpd_register_uri_handler(_httpd, &u);
    };
    reg("/",        HTTP_GET,    h_consoleHtml);   // 文件管理页
    reg("/console", HTTP_GET,    h_consoleHtml);   // 旧地址保留，同一页面
    reg("/files",   HTTP_GET,    h_files);
    reg("/file",    HTTP_GET,    h_fileGet);
    reg("/file",    HTTP_DELETE, h_fileDelete);
    reg("/upload",  HTTP_POST,   h_upload);
    reg("/ota",     HTTP_POST,   h_ota);
    ramCaptureRegister(_httpd);
}

void OtaManager::beginWebServer() {
    if (_consoleActive) return;
    // 不再要求 STA 已连上：常开热点那一路（192.168.4.1）必须始终可烧录。

    if (!_httpd) {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.server_port = 80;
        config.max_uri_handlers = 20;
        config.uri_match_fn = httpd_uri_match_wildcard;
        config.stack_size = 8192;
        config.lru_purge_enable = true;
        if (httpd_start(&_httpd, &config) != ESP_OK) {
            g_dbg->println("HTTP: server start failed.");
            return;
        }
        registerRoutes();
    }

    _consoleActive = true;
    char ip[16];
    getLocalIpStr(ip, sizeof(ip));
    g_dbg->print("HTTP: http://");
    g_dbg->print(ip);
    g_dbg->println("  (文件管理 / OTA 升级)");
}
