unsigned char truong_hop = 0; 
#define cho_sac       1
#define sac_nguoc_cuc 2
#define dang_sac      3
#define sac_day       4

int led_xanh_2 = 11;
int led_xanh_1 = 10;
int led_vang_2 = 9;
int led_vang_1 = 8;
int led_do_1 = 7;
int led_do_2 = 12;
int relay = 4;
int input_nhan_acquy = 6;
int input_pin_sac_nguoc_cuc = 5;

unsigned char dung_luong_acquy = 0;
unsigned dong_dien_max = 800;
unsigned dong_dien_min = 510;

void setup(){
  pinMode(led_do_1 ,OUTPUT);
  pinMode(led_vang_1 ,OUTPUT);
  pinMode(led_vang_2 ,OUTPUT);
  pinMode(led_xanh_1 ,OUTPUT);
  pinMode(led_xanh_2 ,OUTPUT);
  pinMode(led_do_2 ,OUTPUT);
  pinMode(relay ,OUTPUT);
  pinMode(input_nhan_acquy ,INPUT); 
  pinMode(input_pin_sac_nguoc_cuc ,INPUT); 
  digitalWrite(relay, LOW);
  Serial.begin(9600);
  truong_hop = cho_sac  ;
  truong_hop = sac_nguoc_cuc;
  truong_hop = dang_sac ;
  truong_hop = sac_day;

}

void loop(){  
  switch (truong_hop){
    case  cho_sac: 
          function_cho_sac();
          break; 
    case  sac_nguoc_cuc: 
          function_sac_nguoc_cuc();
          break; 
    case  dang_sac : 
          function_dang_sac();
          break; 
     case sac_day : 
          function_sac_day();
          break; 
  } 
}

//cho sac pin
void function_cho_sac()
{
  unsigned char i;
  unsigned dong_dien;
  {
    digitalWrite(led_do_1, HIGH);
    digitalWrite(led_vang_1, HIGH);
    digitalWrite(led_vang_2, HIGH);
    digitalWrite(led_xanh_1, HIGH);
    digitalWrite(led_xanh_2, HIGH);
    delay(2000);

    digitalWrite(led_do_1, LOW);
    digitalWrite(led_vang_1, LOW);
    digitalWrite(led_vang_2, LOW);
    digitalWrite(led_xanh_1, LOW);
    digitalWrite(led_xanh_2, LOW);
    delay(2000);
  }

   //kiem tra nguoc cuc
  for(i=0;i<=5;i++)
  {
   if(digitalRead(input_pin_sac_nguoc_cuc) == 0)
   {  
    delay(10);    
    truong_hop = sac_nguoc_cuc;
   }
   else
   { 
    break;    
   }    
  } 

  for(i=0;i<=5;i++)
  {
    if(digitalRead(input_nhan_acquy) == 0)
    {  
      delay(10);    
      truong_hop = dang_sac;
    }
    else
    { 
      break;    
    }    
  }    
}

//dang sac
void function_dang_sac(){
  unsigned i;
  unsigned dong_dien, dong_dien_tb;
     dong_dien = 0;
     digitalWrite(relay,HIGH);
    //dung luong pin
    dong_dien = 0;
    for(i=0;i<=100;i++)
    {
        dong_dien = analogRead(A0)+dong_dien;
    }
    dong_dien_tb = dong_dien/100;
    dung_luong_acquy = (unsigned char)((dong_dien_max - dong_dien_tb*1000)*100/(dong_dien_max - dong_dien_min));
    dong_dien = 0;
    if((dung_luong_acquy >= 0)&&(dung_luong_acquy < 20))
    { 
      digitalWrite(7,HIGH);
    }
    else if((dung_luong_acquy >= 20)&&(dung_luong_acquy < 50))
    { 
      digitalWrite(7,HIGH);
      digitalWrite(8,HIGH);
    }
    else if((dung_luong_acquy >= 50)&&(dung_luong_acquy < 80))
    { 
      digitalWrite(7,HIGH);
      digitalWrite(8,HIGH);
      digitalWrite(9,HIGH);
    }
   else if((dung_luong_acquy >= 80)&&(dung_luong_acquy < 95))
    { 
      digitalWrite(7,HIGH);
      digitalWrite(8,HIGH);
      digitalWrite(9,HIGH);
      digitalWrite(10,HIGH);
    }
     else (dung_luong_acquy >= 95) ;      //acquy sạc đầy 
     { 
        truong_hop = sac_day;
      }
}

//sac day
void function_sac_day(){
  digitalWrite(relay,LOW);
  digitalWrite(11,HIGH);
  digitalWrite(10,HIGH);
  digitalWrite(9,HIGH);
  digitalWrite(8,HIGH);
  digitalWrite(7,HIGH);
  // tháo các cực của Acquy ra
   if(digitalRead(input_nhan_acquy) == 1 && digitalRead(input_pin_sac_nguoc_cuc) == 1)
   {
       truong_hop = cho_sac;      
    }
}


//sac nguoc cuc
void function_sac_nguoc_cuc(){
         digitalWrite(led_do_2, HIGH);
         delay(500);
         digitalWrite(led_do_2, LOW);
         delay(500);
         digitalWrite(relay, LOW);         
    if(digitalRead(input_pin_sac_nguoc_cuc) == 1)
    {     
       truong_hop = cho_sac;   
    }
}
