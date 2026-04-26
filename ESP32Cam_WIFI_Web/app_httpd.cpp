
#include <WiFi.h>
#include "esp_wifi.h"
#include <esp32-hal-ledc.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "Arduino.h"

extern int gpLed;
byte txdata[4] = {0xAB, 0, 0, 0xFF};//{0xAB, 25, 0, 0xFF};{0xAB, 29, 0, 0xFF};{0xAB, 30, 0, 0xFF};
const int Forward       = 163;                               // 前进
const int Backward      = 92;                              // 后退
const int Turn_Left     = 106;                              // 左平移
const int Turn_Right    = 149;                              // 右平移
const int Top_Left      = 34;                               // 左上移动
const int Bottom_Left   = 72;                              // 左下移动
const int Top_Right     = 129;                               // 右上移动
const int Bottom_Right  = 20;                               // 右下移动
const int Stop          = 0;                                // 停止
const int Contrarotate  = 83;                              // 逆时针旋转
const int Clockwise     = 172;                               // 顺时针旋转
const int Model1        = 25;                               // 模式1
const int Model2        = 26;                               // 模式2
const int Model3        = 27;                               // 模式3
const int Model4        = 28;                               // 模式4
const int Speed         = 29;                               // 速度
const int Servo         = 30;                               // 舵机
const int MotorLeft     = 230;                              // 舵机左转
const int MotorRight    = 231;                              // 舵机右转

typedef struct {
        httpd_req_t *req;
        size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len){
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if(!index){
        j->len = 0;
    }
    if(httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK){
        return 0;
    }
    j->len += len;
    return len;
}

static esp_err_t capture_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    int64_t fr_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");

    size_t out_len, out_width, out_height;
    uint8_t * out_buf;
    bool s;
    {
        size_t fb_len = 0;
        if(fb->format == PIXFORMAT_JPEG){
            fb_len = fb->len;
            res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
        } else {
            jpg_chunking_t jchunk = {req, 0};
            res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk)?ESP_OK:ESP_FAIL;
            httpd_resp_send_chunk(req, NULL, 0);
            fb_len = jchunk.len;
        }
        esp_camera_fb_return(fb);
        int64_t fr_end = esp_timer_get_time();
        Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start)/1000));
        return res;
    }

    bool image_matrix = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
    if (!image_matrix) {
        esp_camera_fb_return(fb);
        Serial.println("dl_matrix3du_alloc failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    
    out_len = fb->width * fb->height * 3;
    out_width = fb->width;
    out_height = fb->height;
    out_buf = (uint8_t*)malloc(out_len);

    s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
    esp_camera_fb_return(fb);
    if(!s){
        free(out_buf);
        Serial.println("to rgb888 failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    jpg_chunking_t jchunk = {req, 0};
    s = fmt2jpg_cb(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, jpg_encode_stream, &jchunk);
    free(out_buf);
    if(!s){
        Serial.println("JPEG compression failed");
        return ESP_FAIL;
    }

    int64_t fr_end = esp_timer_get_time();
    return res;
}

static esp_err_t stream_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;
    char * part_buf[64];

    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    while(true){
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            res = ESP_FAIL;
        } else {
             {
                if(fb->format != PIXFORMAT_JPEG){
                    bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                    esp_camera_fb_return(fb);
                    fb = NULL;
                    if(!jpeg_converted){
                        Serial.println("JPEG compression failed");
                        res = ESP_FAIL;
                    }
                } else {
                    _jpg_buf_len = fb->len;
                    _jpg_buf = fb->buf;
                }
            }
        }
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if(fb){
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        } else if(_jpg_buf){
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if(res != ESP_OK){
            break;
        }
        /*int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        Serial.printf("MJPG: %uB %ums (%.1ffps)\r\n",
            (uint32_t)(_jpg_buf_len),
            (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time           
        );*/
    }

    last_frame = 0;
    return res;
}

enum state {fwd,rev,stp};
state actstate = stp;

static esp_err_t cmd_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;
    char variable[32] = {0,};
    char value[32] = {0,};

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if(!buf){
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
                httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
            } else {
                free(buf);
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
        } else {
            free(buf);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        free(buf);
    } else {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    int val = atoi(value);
    sensor_t * s = esp_camera_sensor_get();
    int res = 0;
    //Serial.println(variable);
    if(!strcmp(variable, "framesize")) 
    {
        Serial.println("framesize");
        if(s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
    }
    else if(!strcmp(variable, "quality")) 
    {
      Serial.println("quality");
      res = s->set_quality(s, val);
    }
    //Remote Control Car 
    //Don't use channel 1 and channel 2
    else if(!strcmp(variable, "flash")) 
    {
      ledcWrite(7,val);
    }  
    else if(!strcmp(variable, "speed")) 
    {
      if      (val > 255) val = 255;
      else if (val <   0) val = 0;       
      txdata[1] = Speed;
      txdata[2] = val;
      Serial.write(txdata, 4);
    }           
    else if(!strcmp(variable, "servo"))
    {
      if      (val > 180) val = 180;
      else if (val < 0) val = 0;     
      //ledcWrite(8,10*val);
      txdata[1] = Servo;
      txdata[2] = val;
      Serial.write(txdata, 4);
    }
    else if(!strcmp(variable, "model")) 
    {
      txdata[2] = 0;
      if (val==1 || val==11)
      {
        txdata[1] = Model1;
        Serial.write(txdata, 4);
      }
      if (val==2 || val==12)
      {
        txdata[1] = Model2;
        Serial.write(txdata, 4);
      }
      if (val==3 || val==13)
      {
        txdata[1] = Model3;
        Serial.write(txdata, 4);
      }
      if (val==4 || val==14)
      {
        txdata[1] = Model4;
        Serial.write(txdata, 4);
      }
    }
    else if(!strcmp(variable, "car")) 
    {  
      txdata[1] = Model1;
      if (val==1) 
      {
        txdata[2] = Forward;
        Serial.write(txdata, 4);
        //Serial.println("Go");
        //httpd_resp_set_type(req, "text/html");
        //return httpd_resp_send(req, "OK", 2);
      }
      else if (val==2) 
      {   
        txdata[2] = Turn_Right;
        Serial.write(txdata, 4);
        //Serial.println("Right");
        //httpd_resp_set_type(req, "text/html");
        //return httpd_resp_send(req, "OK", 2);
      }
      else if (val==3) 
      {
        txdata[2] = Stop;
        Serial.write(txdata, 4);
        //Serial.println("Stop");
        actstate = stp;       
        //httpd_resp_set_type(req, "text/html");
        //return httpd_resp_send(req, "OK", 2); 
      }
      else if (val==4) 
      {
        txdata[2] = Turn_Left;
        Serial.write(txdata, 4);
        //Serial.println("Left");
        //httpd_resp_set_type(req, "text/html");
        //return httpd_resp_send(req, "OK", 2);        
      }
      else if (val==5) 
      {
        txdata[2] = Backward;
        Serial.write(txdata, 4);
        //Serial.println("Back");  
        actstate = rev;      
        //httpd_resp_set_type(req, "text/html");
        //return httpd_resp_send(req, "OK", 2);              
      }
      else if (val==6) 
      {
        txdata[2] = Top_Left;
        Serial.write(txdata, 4);
        //Serial.println("Back");       
        //httpd_resp_set_type(req, "text/html");
        //return httpd_resp_send(req, "OK", 2);              
      }
      else if (val==7) 
      {
        txdata[2] = Top_Right;
        Serial.write(txdata, 4);
        //Serial.println("Back");      
        //httpd_resp_set_type(req, "text/html");
        //return httpd_resp_send(req, "OK", 2);              
      }
      else if (val==8) 
      {
        txdata[2] = Bottom_Left;
        Serial.write(txdata, 4);
        //Serial.println("Back");      
        //httpd_resp_set_type(req, "text/html");
        //return httpd_resp_send(req, "OK", 2);              
      }
      else if (val==9) 
      {
        txdata[2] = Bottom_Right;
        Serial.write(txdata, 4);
        //Serial.println("Back");       
        //httpd_resp_set_type(req, "text/html");
        //return httpd_resp_send(req, "OK", 2);              
      }
      else if (val==10) 
      {
        txdata[2] = Clockwise;
        Serial.write(txdata, 4);
        //Serial.println("Back");      
        //httpd_resp_set_type(req, "text/html");
        //return httpd_resp_send(req, "OK", 2);              
      }
      else if (val==11) 
      {
        txdata[2] = Model1;
        Serial.write(txdata, 4);
        //Serial.println("Back");       
        //httpd_resp_set_type(req, "text/html");
        //return httpd_resp_send(req, "OK", 2);              
      }
      else if (val==12) 
      {
        txdata[2] = Model2;
        Serial.write(txdata, 4);
        //Serial.println("Back");     
        //httpd_resp_set_type(req, "text/html");
        //return httpd_resp_send(req, "OK", 2);              
      }
      else if (val==13) 
      {
        txdata[2] = Model3;
        Serial.write(txdata, 4);
        //Serial.println("Back");      
        //httpd_resp_set_type(req, "text/html");
        //return httpd_resp_send(req, "OK", 2);              
      }
      else if (val==14) 
      {
        txdata[2] = Model4;
        Serial.write(txdata, 4);
        //Serial.println("Back");      
        //httpd_resp_set_type(req, "text/html");
        //return httpd_resp_send(req, "OK", 2);              
      }
      else if (val==15) 
      {
        txdata[2] = Contrarotate;
        Serial.write(txdata, 4);
        //Serial.println("Back");      
        //httpd_resp_set_type(req, "text/html");
        //return httpd_resp_send(req, "OK", 2);              
      }
    }        
    else 
    { 
      Serial.println("variable");
      res = -1; 
    }

    if(res){ return httpd_resp_send_500(req); }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t status_handler(httpd_req_t *req){
    static char json_response[1024];

    sensor_t * s = esp_camera_sensor_get();
    char * p = json_response;
    *p++ = '{';
    
    int rssi = 0;
    int num = WiFi.softAPgetStationNum();
    if(num > 0) {
        wifi_sta_list_t sta;
        memset(&sta, 0, sizeof(wifi_sta_list_t));
        esp_wifi_ap_get_sta_list(&sta);
        if(sta.num > 0) {
            rssi = (int)sta.sta[0].rssi;
        }
    }
    p+=sprintf(p, "\"rssi\":%d,\"num\":%d,\"framesize\":%u,\"quality\":%u}", rssi, num, s->status.framesize, s->status.quality);
    *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!doctype html>
<html>
    <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
        <title>ESP32-CAM Robot</title>
        <style>
            :root {
                --bg-primary: #1a1a2e;
                --bg-secondary: #16213e;
                --bg-gradient: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
                --accent: #667eea;
                --accent-secondary: #764ba2;
                --text-primary: #ffffff;
                --text-secondary: #888888;
                --success: #00ff00;
            }
            *{box-sizing:border-box;margin:0;padding:0;-webkit-user-select:none;-webkit-touch-callout:none}
            body{background:var(--bg-gradient);font-family:'Segoe UI',sans-serif;color:var(--text-primary);min-height:100vh;overflow-x:hidden}
            
            .header{background:rgba(0,0,0,0.5);padding:6px 12px;display:flex;justify-content:space-between;align-items:center;border-radius:15px;margin-bottom:1px;font-size:11px;font-family:monospace}
            .status-left{display:flex;align-items:center;gap:4px;font-size:11px}
            .status-dot{width:6px;height:6px;border-radius:50%;background:var(--success);animation:pulse 2s infinite}
            .mode-right{font-size:11px;font-weight:600;color:#0f0}
            
            
            .video-section{padding:4px;text-align:center}
            .video-container{position:relative;width:100%;max-width:640px;margin:0 auto;border-radius:16px;overflow:hidden;background:#000;aspect-ratio:4/3;display:none}
            .video-container.show{display:block}
            .video-container img{width:100%;height:100%;object-fit:cover}
            .video-controls{display:flex;justify-content:center;gap:10px;padding:8px;background:rgba(0,0,0,0.6);border-radius:0 0 16px 16px}
            .btn{background:rgba(255,255,255,0.15);border:none;color:#fff;padding:5px 14px;border-radius:25px;font-size:12px;cursor:pointer;transition:all 0.2s;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none}
            .btn:active,.btn.active{background:var(--accent)}
            .btn:active{background:var(--accent);transform:scale(0.95)}
            .btn.active{background:var(--accent)}
            
            .joystick-section{margin-top:-4px;padding:4px 16px;text-align:center}
            .joystick-area{position:relative;width:220px;height:220px;margin:0 auto;background:rgba(0,0,0,0.4);border-radius:40px;padding:12px;border:2px solid var(--accent)}
            .direction-grid{display:grid;grid-template-columns:repeat(3,1fr);grid-template-rows:repeat(3,1fr);gap:8px;height:100%}
            .grid-cell{background:rgba(255,255,255,0.1);border-radius:16px;display:flex;align-items:center;justify-content:center;font-size:1.4rem;color:#eceff8;cursor:pointer;transition:all 0.07s;border:1px solid rgba(255,255,255,0.15);-webkit-user-select:none;user-select:none;-webkit-tap-highlight-color:transparent;-webkit-touch-callout:none}
            .grid-cell:active,.grid-cell.active{background:var(--accent);color:#fff}
            .grid-cell.center{background:rgba(0,0,0,0.3);box-shadow:inset 0 0 0 1px var(--accent);cursor:default;color:#666}
            
            .turn-btn{position:absolute;width:45px;height:80px;background:linear-gradient(135deg,#f093fb,#f5576c);border:none;color:#fff;border-radius:22px 6px 6px 22px;font-size:11px;font-weight:bold;cursor:pointer;top:50%;transform:translateY(-50%);z-index:5;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none;display:flex;flex-direction:column;align-items:center;justify-content:center;line-height:1.2}
            .turn-btn span{display:block}
            .turn-btn:active,.turn-btn.active{background:linear-gradient(135deg,#ff6b6b,#ee5a5a);transform:translateY(-50%) translateY(2px)}
            .turn-btn.left{left:-55px}
            .turn-btn.right{right:-55px;border-radius:6px 22px 22px 6px}
            
            .turn-labels{position:absolute;bottom:5px;left:0;right:0;display:flex;justify-content:space-around}
            
            @media(max-width:380px){
                .settings-section{padding:0 16px 2px}
                .setting-row{margin-bottom:2px}
                .joystick-area{width:180px;height:180px;padding:8px;border-radius:30px}
                .direction-grid{gap:5px}
                .grid-cell{font-size:1.1rem;border-radius:12px}
                .turn-btn{width:35px;height:60px;font-size:10px}
                .turn-btn.left{left:-45px}
                .turn-btn.right{right:-45px}
            }
            
.settings-section{padding:0 16px 4px}
            .setting-row{background:rgba(255,255,255,0.08);border-radius:40px;padding:6px 12px;display:flex;align-items:center;gap:10px;border:1px solid rgba(255,255,255,0.1);margin-bottom:4px}
            .setting-label{font-size:11px;font-weight:500;width:45px;flex-shrink:0;white-space:nowrap}
            .setting-slider{flex:1;height:4px;-webkit-appearance:none;background:linear-gradient(90deg,var(--accent),var(--accent-secondary));border-radius:5px}
            .setting-slider::-webkit-slider-thumb{-webkit-appearance:none;width:18px;height:18px;background:#fff;border-radius:50%;cursor:pointer;border:2px solid var(--accent)}
            .setting-value{background:rgba(0,0,0,0.5);padding:4px 10px;border-radius:25px;font-size:12px;width:45px;text-align:center}
            
            .mode-section{padding:1px 16px 8px}
            .mode-buttons{display:flex;gap:8px;flex-wrap:wrap}
            .mode-btn{flex:1;min-width:70px;background:rgba(255,255,255,0.1);border:1px solid rgba(255,255,255,0.2);color:#fff;padding:8px 4px;border-radius:30px;font-size:11px;font-weight:600;cursor:pointer;transition:all 0.2s;text-align:center;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none}
            .mode-btn:active,.mode-btn.selected{background:var(--accent);border-color:transparent}
            
            @keyframes pulse{0%,100%{opacity:1}50%{opacity:0.5}}
        </style>
    </head>
    <body onload="document.addEventListener('touchend',function(e){if(e.target.classList.contains('grid-cell')||e.target.classList.contains('turn-btn'))release();});document.addEventListener('pointerup',function(e){if(e.target.classList.contains('grid-cell')||e.target.classList.contains('turn-btn'))release();});">
        <div class="header">
            <div class="status-left">📡 <span id="rssi-val">...</span></div>
            <div class="mode-right" id="current-mode">🎮 Mode: Free Control</div>
        </div>
        
        <div class="video-section">
            <div class="video-container" id="video-container"><img id="stream" src="" alt="Camera"></div>
            <div class="video-controls">
                <button class="btn" id="btn-start" oncontextmenu="return false" onclick="startStream()">▶ Start</button>
                <button class="btn" id="btn-capture" oncontextmenu="return false" onclick="captureFrame()">📸 Snapshot</button>
                <button class="btn" id="btn-stop" oncontextmenu="return false" onclick="stopStream()">⏹ Stop</button>
            </div>
        </div>
        
        <div class="settings-section">
            <div class="setting-row">
                <span class="setting-label">🔄 Servo</span>
                <input type="range" class="setting-slider" id="servo" min="0" max="180" value="90" oninput="updateSetting('servo',this.value)">
                <span class="setting-value" id="servo-val">90</span>
            </div>
            <div class="setting-row">
                <span class="setting-label">⚡ Speed</span>
                <input type="range" class="setting-slider" id="speed" min="150" max="255" value="220" oninput="updateSetting('speed',this.value)">
                <span class="setting-value" id="speed-val">220</span>
            </div>
            <div class="setting-row">
                <span class="setting-label">💡 LED</span>
                <input type="range" class="setting-slider" id="flash" min="0" max="255" value="0" onchange="try{fetch(document.location.origin+'/control?var=flash&val='+this.value);}catch(e){}">
                <span class="setting-value" id="led-val">0</span>
            </div>
        </div>
        
<div class="joystick-section">
            <div class="joystick-area" oncontextmenu="return false">
                <div class="direction-grid">
                    <div class="grid-cell" id="btn-ul" oncontextmenu="return false" ontouchstart="event.preventDefault();press(6)" ontouchend="release()">↖</div>
                    <div class="grid-cell" id="btn-up" oncontextmenu="return false" ontouchstart="event.preventDefault();press(1)" ontouchend="release()">↑</div>
                    <div class="grid-cell" id="btn-ur" oncontextmenu="return false" ontouchstart="event.preventDefault();press(7)" ontouchend="release()">↗</div>
                    <div class="grid-cell" id="btn-left" oncontextmenu="return false" ontouchstart="event.preventDefault();press(4)" ontouchend="release()">←</div>
                    <div class="grid-cell center" oncontextmenu="return false" ontouchstart="event.preventDefault();release()" ontouchend="release()">⚫</div>
                    <div class="grid-cell" id="btn-right" oncontextmenu="return false" ontouchstart="event.preventDefault();press(2)" ontouchend="release()">→</div>
                    <div class="grid-cell" id="btn-dl" oncontextmenu="return false" ontouchstart="event.preventDefault();press(8)" ontouchend="release()">↙</div>
                    <div class="grid-cell" id="btn-down" oncontextmenu="return false" ontouchstart="event.preventDefault();press(5)" ontouchend="release()">↓</div>
                    <div class="grid-cell" id="btn-dr" oncontextmenu="return false" ontouchstart="event.preventDefault();press(9)" ontouchend="release()">↘</div>
                </div>
                <button class="turn-btn left" id="btn-ccw" oncontextmenu="return false" ontouchstart="event.preventDefault();press(15)" ontouchend="release()"><span>Left</span><span>↺</span></button>
                <button class="turn-btn right" id="btn-cw" oncontextmenu="return false" ontouchstart="event.preventDefault();press(10)" ontouchend="release()"><span>Right</span><span>↻</span></button>
            </div>
        </div>
        
        <div class="mode-section">
            <div class="mode-buttons">
                <button class="mode-btn selected" oncontextmenu="return false" onclick="setMode(1)">Free Control</button>
                <button class="mode-btn" oncontextmenu="return false" onclick="setMode(2)">Obstacle</button>
                <button class="mode-btn" oncontextmenu="return false" onclick="setMode(3)">Following</button>
                <button class="mode-btn" oncontextmenu="return false" onclick="setMode(4)">Line Trace</button>
            </div>
        </div>
        
        <script>
            const BASE_URL = location.origin;
            let currentDir = 3;
            
            function updateStatus(){
                fetch(BASE_URL+'/status').then(r=>r.json()).then(d=>{
                    var el = document.getElementById('rssi-val');
                    if(d.rssi && d.rssi > -120) {
                        el.textContent = 'RSSI ' + d.rssi + ' dBm';
                        el.style.color = d.rssi > -50 ? '#0f0' : (d.rssi > -70 ? '#ff0' : '#f00');
                    } else if(d.num > 0) {
                        el.textContent = 'RSSI no signal';
                        el.style.color = '#f00';
                    } else {
                        el.textContent = '...';
                        el.style.color = '#fff';
                    }
                }).catch(()=>{});
            }
            setInterval(updateStatus,5000);
            updateStatus();
            
            function sendCmd(cmd,val){fetch(`${BASE_URL}/control?var=${cmd}&val=${val}`)}
            
            function startStream(){document.getElementById('stream').src=BASE_URL+':81/stream';document.getElementById('video-container').classList.add('show');document.getElementById('btn-start').classList.add('active')}
            function stopStream(){document.getElementById('stream').src='';document.getElementById('video-container').classList.remove('show');document.querySelectorAll('.video-controls .btn').forEach(b=>b.classList.remove('active'))}
            function captureFrame(){document.getElementById('stream').src=BASE_URL+'/capture?_cb='+Date.now();document.getElementById('video-container').classList.add('show')}
            
            function updateSetting(id,val){
                document.getElementById(id+'-val').textContent=val;
                sendCmd(id,val);
            }
            
            function setMode(val){
                document.querySelectorAll('.mode-btn').forEach(b=>b.classList.remove('selected'));
                document.querySelectorAll('.mode-btn')[val-1].classList.add('selected');
                const modes=['Free Control','Obstacle','Following','Line Trace'];
                document.getElementById('current-mode').textContent='🎮 Mode: '+modes[val-1];
                sendCmd('model',val);
}
            
function press(dir){
                if(currentDir === 3 && dir !== 3){
                    document.querySelectorAll('.mode-btn')[0].classList.add('selected');
                    document.querySelectorAll('.mode-btn').forEach((b,i)=>{if(i>0)b.classList.remove('selected')});
                    document.getElementById('current-mode').textContent='🎮 Mode: Free Control';
                }
                sendCmd('car',dir);
                currentDir = dir;
                highlightArrow(dir);
                if(navigator.vibrate) navigator.vibrate(15);
            }
            
            function release(){
                sendCmd('car',3);
                currentDir = 3;
                clearArrows();
                if(navigator.vibrate) navigator.vibrate(10);
            }
            
            function highlightArrow(dir){
                clearArrows();
                var map = {1:'up',2:'right',4:'left',5:'down',6:'ul',7:'ur',8:'dl',9:'dr',10:'cw',15:'ccw'};
                var idx = map[dir];
                if(idx){
                    var cell = document.getElementById('btn-'+idx);
                    if(cell) cell.classList.add('active');
                }
            }
            
            function clearArrows(){
                var btns = document.querySelectorAll('.grid-cell, .turn-btn');
                for(var i=0;i<btns.length;i++){
                    btns[i].classList.remove('active');
                }
            }
        </script>
    </body>
</html>
)rawliteral";

static esp_err_t index_handler(httpd_req_t *req){
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

void startCameraServer()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    
    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t status_uri = {
        .uri       = "/status",
        .method    = HTTP_GET,
        .handler   = status_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t cmd_uri = {
        .uri       = "/control",
        .method    = HTTP_GET,
        .handler   = cmd_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t capture_uri = {
        .uri       = "/capture",
        .method    = HTTP_GET,
        .handler   = capture_handler,
        .user_ctx  = NULL
    };

   httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };
    
    Serial.printf("Starting web server on port: '%d'\n", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
        httpd_register_uri_handler(camera_httpd, &status_uri);
        httpd_register_uri_handler(camera_httpd, &capture_uri);
    }

    config.server_port += 1;
    config.ctrl_port += 1;
    Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}
