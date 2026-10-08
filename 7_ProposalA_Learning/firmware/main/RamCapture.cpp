#include "RamCapture.h"
#include "DepthController.h"
#include "ProposalStudentConfig.h"
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include "cJSON.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {
constexpr uint32_t INTERVAL_US=50000, MAX_SECONDS=120, RESERVE_BYTES=65536;
SemaphoreHandle_t mutex=nullptr;
std::atomic<bool> recording{false};
CaptureRecord* records=nullptr;
uint32_t count=0, capacity=0, durationSeconds=0, missed=0, sequence=0, previousStamp=0;
uint64_t startedUs=0;
std::atomic<uint32_t> nextMs{0};
const char* state="idle";

class Lock {
public:
    explicit Lock(TickType_t ticks=0): ok(mutex && xSemaphoreTake(mutex,ticks)==pdTRUE) {}
    ~Lock(){if(ok)xSemaphoreGive(mutex);}
    bool ok;
};
esp_err_t error(httpd_req_t* req,const char* status,const char* message){
    httpd_resp_set_status(req,status); httpd_resp_set_type(req,"text/plain");
    return httpd_resp_sendstr(req,message);
}
uint32_t crc32(const uint8_t* data,size_t bytes){
    uint32_t crc=0xffffffffU;
    for(size_t i=0;i<bytes;++i){
        crc^=data[i];
        for(int b=0;b<8;++b)crc=(crc>>1)^((crc&1U)?0xedb88320U:0U);
    }
    return crc^0xffffffffU;
}
esp_err_t statusHandler(httpd_req_t* req){
    Lock lock(pdMS_TO_TICKS(100));
    if(!lock.ok)return error(req,"503 Service Unavailable","recorder busy");
    char text[512];
    std::snprintf(text,sizeof(text),"{\"schema\":1,\"state\":\"%s\",\"recording\":%s,\"count\":%u,\"capacity\":%u,\"duration_s\":%u,\"interval_us\":50000,\"missed_slots\":%u,\"free_heap_bytes\":%u,\"largest_block_bytes\":%u,\"actuator_feedback\":false,\"firmware\":\"hybrid_mpc_v1\",\"student_embedded\":true,\"student_enabled\":%s,\"student_ready\":%s}",
        state,recording.load()?"true":"false",static_cast<unsigned>(count),
        static_cast<unsigned>(capacity),static_cast<unsigned>(durationSeconds),static_cast<unsigned>(missed),
        static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_8BIT)),
        static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)),
        proposal_model_config::kEnabled ? "true" : "false",
        proposalStudentRuntimeReady() ? "true" : "false");
    httpd_resp_set_type(req,"application/json");return httpd_resp_sendstr(req,text);
}
esp_err_t startHandler(httpd_req_t* req){
    if(req->content_len<=0 || req->content_len>=128)return error(req,"400 Bad Request","JSON duration_s required (1..120)");
    char body[128]={};int received=0;
    while(received<req->content_len){
        int n=httpd_req_recv(req,body+received,req->content_len-received);
        if(n<=0)return error(req,"408 Request Timeout","incomplete start request");
        received+=n;
    }
    cJSON* json=cJSON_Parse(body);
    const cJSON* duration=json?cJSON_GetObjectItemCaseSensitive(json,"duration_s"):nullptr;
    double seconds=cJSON_IsNumber(duration)?duration->valuedouble:0;
    if(!std::isfinite(seconds)||seconds<1||seconds>MAX_SECONDS||seconds!=std::floor(seconds)){
        cJSON_Delete(json);return error(req,"400 Bad Request","duration_s must be integer 1..120");
    }
    cJSON_Delete(json);
    {
        Lock lock(pdMS_TO_TICKS(100));
        if(!lock.ok)return error(req,"503 Service Unavailable","recorder busy");
        if(records||recording.load())return error(req,"409 Conflict","download and explicitly clear existing capture first");
        uint32_t desired=static_cast<uint32_t>(seconds)*20U;
        size_t bytes=desired*sizeof(CaptureRecord);
        size_t freeBytes=heap_caps_get_free_size(MALLOC_CAP_8BIT);
        if(freeBytes<bytes+RESERVE_BYTES || heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)<bytes)
            return error(req,"507 Insufficient Storage","not enough RAM; request a shorter duration");
        records=static_cast<CaptureRecord*>(heap_caps_malloc(bytes,MALLOC_CAP_8BIT));
        if(!records)return error(req,"507 Insufficient Storage","RAM allocation failed");
        count=0;capacity=desired;durationSeconds=static_cast<uint32_t>(seconds);missed=0;sequence=0;previousStamp=0;
        startedUs=static_cast<uint64_t>(esp_timer_get_time());nextMs.store(static_cast<uint32_t>(startedUs/1000U));
        state="recording";recording.store(true,std::memory_order_release);
    }
    return statusHandler(req);
}
esp_err_t stopHandler(httpd_req_t* req){
    {Lock lock(pdMS_TO_TICKS(100));if(!lock.ok)return error(req,"503 Service Unavailable","recorder busy");
     recording.store(false);if(records)state="ready";}
    return statusHandler(req); // This stops LOGGING ONLY, never robot motion.
}
esp_err_t clearHandler(httpd_req_t* req){
    {Lock lock(pdMS_TO_TICKS(100));if(!lock.ok)return error(req,"503 Service Unavailable","recorder busy");
     if(recording.load())return error(req,"409 Conflict","stop capture before clearing");
     heap_caps_free(records);records=nullptr;count=capacity=durationSeconds=missed=0;state="idle";}
    return statusHandler(req);
}
esp_err_t downloadHandler(httpd_req_t* req){
    Lock lock(pdMS_TO_TICKS(100));if(!lock.ok)return error(req,"503 Service Unavailable","recorder busy");
    if(recording.load())return error(req,"409 Conflict","capture still recording");
    if(!records||!count)return error(req,"404 Not Found","no capture data");
    CaptureHeader header{};std::memcpy(header.magic,"SQDCAP1",7);
    header.version=1;header.recordBytes=sizeof(CaptureRecord);header.count=count;
    header.intervalUs=INTERVAL_US;header.missedSlots=missed;
    header.crc32=crc32(reinterpret_cast<const uint8_t*>(records),count*sizeof(CaptureRecord));
    header.kp=depth_cfg::KP;header.ki=depth_cfg::KI;header.kd=depth_cfg::KD;
    header.sinkGain=depth_cfg::G_SINK;header.riseGain=depth_cfg::G_RISE;
    header.restAccel=depth_cfg::A_REST_REF;header.alpha=depth_cfg::ALPHA;header.referenceDepth=depth_cfg::DEPTH_REF;
    httpd_resp_set_type(req,"application/octet-stream");
    httpd_resp_set_hdr(req,"Content-Disposition","attachment; filename=capture.sqdcap");
    if(httpd_resp_send_chunk(req,reinterpret_cast<const char*>(&header),sizeof(header))!=ESP_OK)return ESP_FAIL;
    const char* bytes=reinterpret_cast<const char*>(records);size_t remaining=count*sizeof(CaptureRecord);
    while(remaining){
        size_t n=remaining>2048?2048:remaining;
        if(httpd_resp_send_chunk(req,bytes,n)!=ESP_OK)return ESP_FAIL;
        bytes+=n;
        remaining-=n;
    }
    return httpd_resp_send_chunk(req,nullptr,0);
}
}
bool ramCaptureDue(uint64_t nowUs){
    return recording.load(std::memory_order_acquire) &&
        static_cast<int32_t>(static_cast<uint32_t>(nowUs/1000U)-nextMs.load())>=0;
}
void ramCapturePush(CaptureRecord record,uint64_t nowUs,uint32_t depthStampMs){
    if(!ramCaptureDue(nowUs))return;
    Lock lock; if(!lock.ok)return; // Never block the 5ms motion loop on HTTP.
    if(!recording.load())return;
    if(nowUs-startedUs>=static_cast<uint64_t>(durationSeconds)*1000000U||count>=capacity){
        recording.store(false);state="ready";return;
    }
    uint32_t scheduled=nextMs.load();
    uint32_t slots=(static_cast<uint32_t>(nowUs/1000U)-scheduled)/(INTERVAL_US/1000U);
    missed+=slots;nextMs.store(scheduled+(slots+1U)*(INTERVAL_US/1000U));
    record.elapsedUs=static_cast<uint32_t>(nowUs-startedUs);
    if(depthStampMs && depthStampMs!=previousStamp){++sequence;previousStamp=depthStampMs;
        if(record.flags&CAP_DEPTH_VALID)record.flags|=CAP_DEPTH_FRESH;}
    record.depthSequence=sequence;records[count++]=record;
}
void ramCaptureRegister(httpd_handle_t server){
    if(!mutex)mutex=xSemaphoreCreateMutex();
    if(!mutex)return;
    const httpd_uri_t routes[]={
        {"/capture/status",HTTP_GET,statusHandler,nullptr},
        {"/capture/start",HTTP_POST,startHandler,nullptr},
        {"/capture/stop",HTTP_POST,stopHandler,nullptr},
        {"/capture/clear",HTTP_POST,clearHandler,nullptr},
        {"/capture/data",HTTP_GET,downloadHandler,nullptr}};
    for(const auto& route:routes)httpd_register_uri_handler(server,&route);
}
