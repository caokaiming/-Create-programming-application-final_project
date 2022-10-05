//*******************************************************************************
//E94084066曹豈銘 final demo code，主要針對原超音波避障模式程式進行修改
//*******************************************************************************
#include <boarddefs.h>
#include <IRremoteInt.h>
#include <ir_Lego_PF_BitStreamEncoder.h>

#include <IRremote.h>

#define MotorA_I1     8  //宣告 I1 接腳
#define MotorA_I2     9  //宣告 I2 接腳
#define MotorB_I3    10  //宣告 I3 接腳
#define MotorB_I4    11  //宣告 I4 接腳
#define MotorA_PWMA    5  //宣告 ENA (PWM調速) 接腳
#define MotorB_PWMB    6  //宣告 ENB (PWM調速) 接腳

#define IR_Recv      3   // 宣告紅外線接收接腳
IRrecv irrecv(IR_Recv);  // 宣告 IRrecv 物件來接收紅外線訊號　
decode_results results;  // 宣告解碼變數


#define IR_Forwards    0xFF629D  // 遙控器方向鍵 上, 前進
#define IR_Back        0xFFA857  // 遙控器方向鍵 下, 後退
#define IR_Stop        0xFF02FD // 遙控器 OK 鍵, 停止
#define IR_Left        0xFF22DD  // 遙控器方向鍵 左, 左轉
#define IR_Right       0xFFC23D  // 遙控器方向鍵 右, 右轉

#define IR_Tracking    0xFF42BD  // 遙控器 * 鍵, 循跡模式
#define IR_Ultrasonic  0xFF52AD  // 遙控器 # 鍵, 超音波避障模式


#define SensorLeft    A0  //宣告 左側感測器 輸入腳   
#define SensorMiddle  A1  //宣告 中間感測器 輸入腳
#define SensorRight   A2  //宣告 右側感測器 輸入腳
int off_track = 0;        //宣告變數

int US_Trig = A3;  //宣告超音波模組 Trig 腳位
int US_Echo = A4;  //宣告超音波模組 Echo 腳位
unsigned long sonar_previous_time = 0;
unsigned long sonar_current_time, sonar_pass_time;
float sonar_duty, sonar_distance = 0;


int lasertrig = A5;
unsigned long laser_on_time = 0;
unsigned long laser_last_time;
bool laserOn= false;

const int isObstaclePin = 2;            
volatile bool isObstacle = false;      
const int ledPin = 13;     

void pinFalled()  
{
 isObstacle = true;
}  


float sonar(unsigned long delayTime){ 
  sonar_current_time = micros();
  sonar_pass_time = sonar_current_time - sonar_previous_time;
  digitalWrite(US_Trig, HIGH);
  if(sonar_pass_time >=delayTime*1000) {
    digitalWrite(US_Trig, LOW);
    sonar_previous_time = micros();
    sonar_duty = pulseIn(US_Echo,HIGH);
    sonar_distance = (sonar_duty/2)/29.4;
    }
  return sonar_distance;
}

void laserfired(unsigned long delayTime) {
  if (!laserOn){
      digitalWrite(lasertrig, HIGH);
      digitalWrite(ledPin, HIGH);
      laser_on_time= micros();
      laserOn=true; 
   }    
  laser_last_time = micros()-laser_on_time;
  if(laser_last_time >= delayTime*1000)
   {
    digitalWrite(lasertrig, LOW);
    digitalWrite(ledPin, LOW);
    laserOn=false;
   } 
}

void advance(int a)    // 小車前進
{
    digitalWrite(MotorA_I1,HIGH);   //馬達（右）順時針轉動
    digitalWrite(MotorA_I2,LOW);
    digitalWrite(MotorB_I3,HIGH);   //馬達（左）逆時針轉動
    digitalWrite(MotorB_I4,LOW);
    delay(a * 100);
}

void turnR(int d)    // 小車右轉
{
    digitalWrite(MotorA_I1,LOW);    //馬達（右）逆時針轉動
    digitalWrite(MotorA_I2,HIGH);
    digitalWrite(MotorB_I3,HIGH);   //馬達（左）逆時針轉動
    digitalWrite(MotorB_I4,LOW);
    delay(d * 10);
}

void turnL(int e)    // 小車左轉
{
    digitalWrite(MotorA_I1,HIGH);   //馬達（右）順時針轉動
    digitalWrite(MotorA_I2,LOW);
    digitalWrite(MotorB_I3,LOW);    //馬達（左）順時針轉動
    digitalWrite(MotorB_I4,HIGH);
    delay(e * 10);
}    

void stopRL(int f)  // 小車停止
{
    digitalWrite(MotorA_I1,HIGH);   //馬達（右）停止轉動
    digitalWrite(MotorA_I2,HIGH);
    digitalWrite(MotorB_I3,HIGH);   //馬達（左）停止轉動
    digitalWrite(MotorB_I4,HIGH);
    delay(f * 100);
}

void back(int g)    // 小車後退
{
    digitalWrite(MotorA_I1,LOW);    //馬達（右）逆時針轉動
    digitalWrite(MotorA_I2,HIGH);
    digitalWrite(MotorB_I3,LOW);    //馬達（左）順時針轉動
    digitalWrite(MotorB_I4,HIGH);
    delay(g * 100);     
}
    

void setup()
{
  Serial.begin(9600); 
  
  pinMode(MotorA_I1,OUTPUT);
  pinMode(MotorA_I2,OUTPUT);
  pinMode(MotorB_I3,OUTPUT);
  pinMode(MotorB_I4,OUTPUT);
  pinMode(MotorA_PWMA,OUTPUT);
  pinMode(MotorB_PWMB,OUTPUT);
  
  pinMode(isObstaclePin, INPUT); //Receive IR Obstacle interrupt Signal
  pinMode(lasertrig, OUTPUT);
  digitalWrite (lasertrig, LOW);

  pinMode(US_Trig, OUTPUT);
  pinMode(US_Echo, INPUT);
 
  pinMode(SensorLeft,   INPUT); 
  pinMode(SensorMiddle, INPUT);
  pinMode(SensorRight,  INPUT);
  
  irrecv.enableIRIn();  
  
  analogWrite(MotorA_PWMA,200);    //設定馬達 (右) 轉速
  analogWrite(MotorB_PWMB,200);    //設定馬達 (左) 轉速
  attachInterrupt (digitalPinToInterrupt (isObstaclePin), pinFalled, FALLING);  // attach interrupt handler
}

void blueToothCommand()
{
  int cmd = Serial.read();  // 讀取藍芽指令
    
  switch(cmd)  // 執行藍芽指令
  {
      case 'B':  // 倒車
      back(5);
      break;
    case 'L':  // 左轉
      turnL(5);
      break;       
    case 'R':  // 右轉
      turnR(5);
      break;     
    case 'F':  // 前進
      advance(5);
      break;        
    case 'S':  // 停止
        stopRL(5);
        break;

    case 'X':  
        break;        
    case 'N':  // 超音波測距查詢
        Serial.print("Ultrasonic Distance:");          // 輸出距離（單位：公分）
        Serial.println(sonar(2));                      // 顯示距離
        break;
  }
}

void loop()
{
    blueToothCommand();  // Read if there is a command from BlueTooth module
   
    if(irrecv.decode(&results)) 
    {
        // 解碼成功，收到一組紅外線訊號
        Serial.print("\r\nIR Code: ");            
        Serial.print(results.value, HEX); 
        switch(results.value)
        {
          case IR_Forwards:
            Serial.print(" Forwards");
            advance(1);
            break;
            
          case IR_Back:
            Serial.print(" Back");
            back(1);
            break;
            
          case IR_Stop:
            Serial.print(" Stop");
            stopRL(1);
            break;
            
          case IR_Left:
            Serial.print(" Left");
            turnL(1);
            break;
            
          case IR_Right:
            Serial.print(" Right");
            turnR(1);
            break;
            
          case IR_Tracking: //Enter Line Tracking mode
            Serial.print(" Tracking Mode");
            
            irrecv.resume();
            
            while(true)
            {
              // 讀取感測器狀態
              int SL = digitalRead(SensorLeft);
              int SM = digitalRead(SensorMiddle);
              int SR = digitalRead(SensorRight);
              
              if((SM == LOW) && (SL == LOW) && (SR == LOW))  // 小車脫離黑線
              {
                analogWrite(MotorA_PWMA,200);    //設定馬達 (右) 轉速
                analogWrite(MotorB_PWMB,200);    //設定馬達 (左) 轉速
				
                if(off_track < 3)
                {
                  switch(off_track)
                  {
                    case 0:
                      back(1);
                      break;
                    
                    case 1:
                      turnR(1);
                      break;
                      
                    case 2:
                      turnL(2);
                      break;
                  }
                  
                  off_track++;
                }
                else
                {
                  stopRL(0);
                }
              }
              else
              {
                off_track = 0;
                               
                if(SM == HIGH)  //中感測器在黑色區域
                {
                  if((SL == LOW) && (SR == HIGH))  // 左白右黑, 車體偏右校正
                  {
                    analogWrite(MotorA_PWMA,200);    //設定馬達 (右) 轉速
                    analogWrite(MotorB_PWMB, 80);    //設定馬達 (左) 轉速
                    advance(0);
                  } 
                  else if((SL == HIGH) && (SR == LOW))  // 左黑右白, 車體偏左校正
                  {
                    analogWrite(MotorA_PWMA, 80);    //設定馬達 (右) 轉速
                    analogWrite(MotorB_PWMB,200);    //設定馬達 (左) 轉速
                    advance(0);
                  }
                  else  // 其他, 直走
                  {
                    analogWrite(MotorA_PWMA,200);    //設定馬達 (右) 轉速
                    analogWrite(MotorB_PWMB,200);    //設定馬達 (左) 轉速
                    advance(0);
                  }
                } 
                else // 中感測器在白色區域, 車體已大幅偏離黑線
                {
                  if((SL == LOW) && (SR == HIGH))  // 左白右黑, 車體快速右轉
                  {
                    analogWrite(MotorA_PWMA,200);    //設定馬達 (右) 轉速
                    analogWrite(MotorB_PWMB,200);    //設定馬達 (左) 轉速
                    turnR(0);
                  }
                  else if((SL == HIGH) && (SR == LOW))  // 左黑右白, 車體快速左轉
                  {
                    analogWrite(MotorA_PWMA,200);    //設定馬達 (右) 轉速
                    analogWrite(MotorB_PWMB,200);    //設定馬達 (左) 轉速
                    turnL(0);
                  }
                }
              }
              
              if(irrecv.decode(&results)) //See if IR remote controller sent any command?
              {
                irrecv.resume();
                
                if(results.value == IR_Stop)
                {
                  Serial.print("\r\nStop Tracking Mode");
                  stopRL(1);
                  analogWrite(MotorA_PWMA,200);    //設定馬達 (右) 轉速
                  analogWrite(MotorB_PWMB,200);    //設定馬達 (左) 轉速
                  break;
                }
              }
            }
            break;
            
          case IR_Ultrasonic:   //主要部分
            Serial.print("Begin Ultrasonic detector");
            
            irrecv.resume();
            
            while (true){
            if(sonar(2) < 10.0 || isObstacle)  
              {
              stopRL(0);   
              while (sonar(2) < 15.0 || digitalRead(isObstaclePin)==LOW){   
                  delay(100);
                  back(3);
                  delay(100);
                  turnL(10);
                  }    
              if (digitalRead(isObstaclePin)!=LOW) isObstacle =false;
              }
            else
              {
                if(laserOn)
                laserfired(500);
                advance(0);                
              }
               
              if(irrecv.decode(&results))
              {
                irrecv.resume();
                
                if(results.value == IR_Stop)
                {
                  Serial.print("\r\nStop Ultrasonic detector");
                  stopRL(1);
                  break;
                }
              }
            }
            break;
            
          default:
          Serial.print(" Unsupported");
        }
    
        irrecv.resume(); // Receive the next value
    }
 }
