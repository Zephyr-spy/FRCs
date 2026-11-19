# 底盤自瞄
==========
>## PID基本定義
-----------
>>### *P:比例控制項。距離目標位置的誤差值，誤差值越大輸出功率越高  
>>### *I:積分控制項。累計過去的誤差值，消除輸出結果與預定值的誤差，使其回到平穩的預計誤差值  
>>### *D:微分控制項。預測未來的誤差趨勢，預先調整以減少震盪  

# FRC 機器人底盤單側驅動子系統（Tank Side Subsystem）設計架構與邏輯說明
---

## 一、 設計邏輯 

### 1. 馬達Follow
* **設計實作**：`followMotorConfig.follow(leader, false);`
* **優點**：底盤單側由雙馬達驅動同一齒輪箱。Follow模式，從動馬達（Follower）複製主動馬達（Leader）的輸出訊號，避免兩個馬達分開控制造成齒輪箱內齒輪互相嚙合磨損。


### 2. 把控制權交給馬達晶片（板載 PID 速度控制）
* **Code位置**：`followMotorConfig.follow(leader, false);`
* **控制邏輯**：我們直接把 PID 控制參數存進 SPARK Max 晶片裡。當呼叫 `setVelocity` 要求固定速度時，馬達晶片會每20毫秒的速度檢查真實速度，並自動補足力量。

### 3. 直接使用物理單位（SI 標準單位轉換）
* **Code位置**：設定 `positionConversionFactor`（位置）與 `velocityConversionFactor`（速度）。
* **控制邏輯**：我們把輪胎的大小（圓周長）和齒輪箱的減速比輸入進去。當呼叫 `getPosition()` 拿到的直接就是「公尺」，`getVelocity()` 就是「速度（m/s）」，取代Encoder的計算馬達圈數。



---



