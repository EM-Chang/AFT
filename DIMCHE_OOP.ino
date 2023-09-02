#include <DFPlayer_Mini_Mp3.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// LiquidCrystal_I2C lcd(0x27,16,2);  // LCD 시작주소 0x27 for a 16 문자 2라인 디스플레이
LiquidCrystal_I2C lcd(0x27, 20, 4);  // LC D 시작주소 0x27 for a 20 문자 4라인 디스플레이
// A4 - SDA  , A5 - SDL , GND , 5V 4선 사용

/* -----사용가능 핀------
  1) 디지털 I/O
  D2,D3,D4,D5,D6,D7,D8,D9,D45,D46
  D0 =RX0, D1 =TX0 D18=TX1 D16=TX2 D14=TX3, D19=RX1 D17=RX2  D15=RX3
  D20=SDA, D21=SCL

  2) 아날로그 I/O
  A8,A9,A10,A11,A12
*/

//------------------------ BUZZER 소리설정 ---------------------------------------

#define ton_ng       2500         // 불합격음
#define NOTE_C5       523          // 도
#define NOTE_D5       587          // 레 
#define NOTE_E5       659          // 미 
#define NOTE_F5       698          // 파
#define NOTE_G5       784          // 솔
#define NOTE_A5       880          // 라
#define NOTE_B5       988          // 시
#define NOTE_C6       1047         // 도
#define NOTE_D6       1175         // 레
#define NOTE_E6       1319         // 미
#define NOTE_F6       1397
#define NOTE_FS6      1480
#define NOTE_G6       1568
#define NOTE_GS6      1661
#define NOTE_A6       1760
#define NOTE_AS6      1865
#define NOTE_B6       1976
#define NOTE_C7       2093
#define NOTE_CS7      2217
#define NOTE_D7       2349
#define NOTE_DS7      2489
#define NOTE_E7       2637
#define NOTE_F7       2794

//--------------------------------------------------------------------------------
#define power          10     // 테스트 전력 ON/OFF OUT PIN
#define discharge      9
#define start_btn      2
#define stop_btn       3

//----------부하 입력 및 LED 점등 출력 핀 설정--------------------------------------
#define Load_doorheat  19

#define Load1_relay    22
#define Load2_relay    24
#define Load3_relay    26
#define Load4_relay    28
#define Load5_relay    30
#define Load6_relay    32
#define Load7_relay    34
#define Load8_relay    36
#define Load9_relay    38

#define Load_comp      53
#define led_1st        23
#define led_2nd        25
#define led_3rd        27
#define led_4th        29
#define led_5th        31 
#define led_6th        33
#define led_7th        35
#define led_8th        37
#define led_9th        39
#define led_comp       40

//----------------댐퍼 및 스탭밸브 핀 설정---------------------------
#define step_a         41
#define step_b         42
#define step_c         43
#define step_d         44
#define damper_1a      45
#define damper_1b      46
#define damper_1c      47
#define damper_1d      48
#define damper_2a      49
#define damper_2b      50
#define damper_2c      51
#define damper_2d      52

// 합격/불합격 LED 점등 및 부저 핀 설정
#define start_led       6
#define ok_led         11
#define ng_led         12
#define BUZZER         13

// 전압감시 핀 설정
#define vol_1          A2  // 처리 로직 없음 
#define vol_2          A3  // 처리 로직 없음

// 옵션SW INPUT 핀
#define opt_1          A4  // opt_value_1
#define opt_2          A5  // opt_value_2
#define opt_3          A6  // opt_value_3
#define opt_4          A7  // opt_value_4

// 전류센서 사용용량 설정
// #define CURRENT 20         // 20A 사용 아직 사용안함

// 시간 측정
unsigned int count_standby = 0;      // 10ms 단위로 countUp
unsigned int count_standby_1 = 0;    // 테스트 function에서 설정된 값을 1s 단위로 countDown -> drcrease_count_1s()
unsigned int count_standby_2 = 0;    // 100ms 단위로 countUp
unsigned int count_standby_3 = 0;    // 1s 단위로 countUp
unsigned int count_comp = 0;
unsigned int count_step_on = 0;

int flag = 0;                       // 1s 단위로 countUp, led 반전용 -> time_siso()
unsigned int count_siso = 0;        // 1s 단위로 countUp -> time_siso()

unsigned int load_on = 0;
unsigned int opt_count = 0;
unsigned int opt_model = 0;
unsigned int push_switch_on = 0;
unsigned int cycle_complete = 0;

//---------------------------------------------------------------------------
int vs_Test_order = 0;
int vs_Sub_order = 0;
int vs_SSub_order = 0;
int vs_step_order = 0;
int vs_option_order = 0;
int test_start_stop = 0;
//-------------------------------------------------------------------------
float input_vol_a0;          // A0 입력되는 전압 아날로그 값 데이터
float input_vol_a1;
float input_vol_a2;
float input_vol_a3;

float cal_vol_a0;            // A0 계산후 출력전압  5V
float cal_vol_a1;
float cal_vol_a2;
float cal_vol_a3;

//------------------------옵션 설정 -------------------------------------------
unsigned int opt_value_1;    // A13 14 15 READ값      
unsigned int opt_value_2;
unsigned int opt_value_3;
unsigned int opt_value_4;

//------------------------부하 11개 사용----------------------------------------
unsigned int relay1_input = 0;        // LOW 동작 확인값 1= O.K  0 : N.G
unsigned int relay2_input = 0;
unsigned int relay3_input = 0;
unsigned int relay4_input = 0;
unsigned int relay5_input = 0;
unsigned int relay6_input = 0;
unsigned int relay7_input = 0;
unsigned int relay8_input = 0;
unsigned int relay9_input = 0;


//----------------------시간 설정------------------------------------------------------
unsigned long past = 0;              // 과거 시간 저장 변수
unsigned long past1 = 0;
unsigned long past2 = 0;
unsigned long past3 = 0;
unsigned long past4 = 0;
unsigned long past5 = 0;
unsigned long past6 = 0;
unsigned long past7 = 0;



//-------------------------LED 동작설정 -----------------------------------------------
void led_on_off()
{
	if (test_start_stop == 1) {
		if (digitalRead(Load1_relay) == LOW)    digitalWrite(led_1st, HIGH);
		else                                   digitalWrite(led_1st, LOW);
		if (digitalRead(Load2_relay) == LOW)    digitalWrite(led_2nd, HIGH);
		else                                   digitalWrite(led_2nd, LOW);
		if (digitalRead(Load3_relay) == LOW)    digitalWrite(led_3rd, HIGH);
		else                                  digitalWrite(led_3rd, LOW);
		if (digitalRead(Load4_relay) == LOW)    digitalWrite(led_4th, HIGH);
		else                                   digitalWrite(led_4th, LOW);
		if (digitalRead(Load5_relay) == LOW)    digitalWrite(led_5th, HIGH);
		else                                  digitalWrite(led_5th, LOW);
		if (digitalRead(Load6_relay) == LOW)    digitalWrite(led_6th, HIGH);
		else                                  digitalWrite(led_6th, LOW);
		if (digitalRead(Load8_relay) == LOW)    digitalWrite(led_8th, HIGH);
		else                                  digitalWrite(led_8th, LOW);
		// LOAD8-1       
		if (opt_model == 1) {
			if (digitalRead(Load7_relay) == LOW)   digitalWrite(led_7th, HIGH);
			else                                  digitalWrite(led_7th, LOW);
		}
		else {
			if (digitalRead(Load_doorheat) == LOW) digitalWrite(led_7th, HIGH);
			else                                  digitalWrite(led_7th, LOW);
		}

		// LOAD9        10번 LED      2021/04/19
		if (digitalRead(Load9_relay) == LOW)   digitalWrite(led_9th, HIGH);
		else                                 digitalWrite(led_9th, LOW);

		// LOAD COMP   11번 LED 
		if (digitalRead(Load_comp) == LOW) {
			digitalWrite(led_comp, HIGH);
			count_comp_1s();
		}
		else {
			digitalWrite(led_comp, LOW);
		}

		// JS 도어 히터 모델의 LED는 5번 보르도와인은 9번 LED를 사용한다    2021/04/19  
	}
}

void load_led_all_off()
{
	digitalWrite(led_1st, LOW);
	digitalWrite(led_2nd, LOW);
	digitalWrite(led_3rd, LOW);
	digitalWrite(led_4th, LOW);
	digitalWrite(led_5th, LOW);
	digitalWrite(led_6th, LOW);
	digitalWrite(led_7th, LOW);
	digitalWrite(led_8th, LOW);
	digitalWrite(led_9th, LOW);
	digitalWrite(led_comp, LOW);
	digitalWrite(ok_led, LOW);
	digitalWrite(ng_led, LOW);
	digitalWrite(start_led, LOW);
	return;
}

void load_led_11_off()
{
	digitalWrite(led_1st, LOW);
	digitalWrite(led_2nd, LOW);
	digitalWrite(led_3rd, LOW);
	digitalWrite(led_4th, LOW);
	digitalWrite(led_5th, LOW);
	digitalWrite(led_6th, LOW);
	digitalWrite(led_7th, LOW);
	digitalWrite(led_8th, LOW);
	digitalWrite(led_9th, LOW);
	digitalWrite(led_comp, LOW);

}

//-----------------시작 정지 버튼 설정 ------------------------------------

void power_start_stop()
{
	if (digitalRead(stop_btn) == LOW)
	{
		digitalWrite(power, LOW);
		delay(200);
		digitalWrite(discharge, LOW);
		count_standby = 0;
		count_standby_1 = 0;
		count_standby_3 = 0;
		load_on = 0;
		count_step_on = 0;
		count_siso = 0;
		count_comp = 0;
		cycle_complete = 0;
		vs_option_order = 0;
		vs_Test_order = 72;
	}
	if (digitalRead(start_btn) == LOW && test_start_stop == 0)
	{
		digitalWrite(power, HIGH);
		digitalWrite(start_led, HIGH);
		count_standby = 0;
		count_standby_1 = 0;
		count_standby_3 = 0;
		count_comp = 0;
		load_on = 0;
		count_step_on = 0;
		count_siso = 0;
		vs_Test_order = 73;
	}
}


// 대기시간을 유효 호출 횟수로 count

void Standby_count() {                     // 10ms 이상 유효, lead/step/damper 테스트 시도 횟수 count용
	unsigned long now = millis();
	if (now - past >= 10) {
		past = now;
		count_standby++;
	}
}

void Standby_count_100ms() {               // 100ms 이상 유효, **사용처 없음
	unsigned long now2 = millis();
	if (now2 - past2 >= 100) {
		past2 = now2;
		count_standby_2++;
	}
}


void Standby_count_1s() {                 // 1s 이상 유효, **사용처 없음
	unsigned long now3 = millis();
	if (now3 - past3 >= 1000) {
		past3 = now3;
		count_standby_3++;
	}
}

void time_siso() {                       // 1s 이상 유효, NG 표시용 역속 동작 (led 반전 flag, count_siso)
	unsigned long now4 = millis();
	if (now4 - past4 >= 1000) {
		past4 = now4;
		flag++;
		count_siso++;
	}
}

void Load_on_count() {                  // 10ms 이상 유효, 테스트 결과 ON 검출 횟수 count용
	unsigned long now1 = millis();
	if (now1 - past1 >= 10) {
		past1 = now1;
		load_on++;
	}
}

void Step_on_count() {                   // 10ms 이상 유효, **사용처 없음
	unsigned long now5 = millis();
	if (now5 - past5 >= 10) {
		past5 = now5;
		count_step_on++;
	}
}

void drcrease_count_1s() {              // AC&DC LOAD OK 표시 hold 시간 countDown 용
	unsigned long now6 = millis();
	if (now6 - past6 >= 1000) {
		past6 = now6;
		count_standby_1--;
	}
}

void count_comp_1s() {                 // Load_comp 동작 검출 count용
	unsigned long now7 = millis();
	if (now7 - past7 >= 1000) {
		past7 = now7;
		count_comp++;
	}
}


// ----------------------핀 SETUP --------------------------

void setup()
{
	// Serial.begin(1200);
	Serial3.begin(9600);
	mp3_set_serial(Serial3);  //set softwareSerial for DFPlayer-mini mp3 module 
	delay(1);  //wait 1ms for mp3 module to set volume
	mp3_set_volume(30);          // value 0~30

	pinMode(power, OUTPUT);
	pinMode(stop_btn, INPUT_PULLUP);
	pinMode(start_btn, INPUT_PULLUP);

	pinMode(Load1_relay, INPUT_PULLUP);      //22,23
	pinMode(led_1st, OUTPUT);
	pinMode(Load2_relay, INPUT_PULLUP);      //24,25
	pinMode(led_2nd, OUTPUT);
	pinMode(Load3_relay, INPUT_PULLUP);      //26,27
	pinMode(led_3rd, OUTPUT);
	pinMode(Load4_relay, INPUT_PULLUP);      //28,29
	pinMode(led_4th, OUTPUT);
	pinMode(Load5_relay, INPUT_PULLUP);      //30,31
	pinMode(led_5th, OUTPUT);
	pinMode(Load6_relay, INPUT_PULLUP);      //32,33
	pinMode(led_6th, OUTPUT);
	pinMode(Load7_relay, INPUT_PULLUP);      //34,35
	pinMode(led_7th, OUTPUT);
	pinMode(Load8_relay, INPUT_PULLUP);      //36,37
	pinMode(led_8th, OUTPUT);
	pinMode(Load9_relay, INPUT_PULLUP);      //38,39
	pinMode(led_9th, OUTPUT);
	pinMode(Load_comp, INPUT_PULLUP);        //53
	pinMode(led_comp, OUTPUT);
	pinMode(Load_doorheat, INPUT_PULLUP);   //20

	pinMode(step_a, INPUT_PULLUP);      //41
	pinMode(step_b, INPUT_PULLUP);      //42
	pinMode(step_c, INPUT_PULLUP);      //43
	pinMode(step_d, INPUT_PULLUP);
	pinMode(damper_1a, INPUT_PULLUP);
	pinMode(damper_1b, INPUT_PULLUP);
	pinMode(damper_1c, INPUT_PULLUP);
	pinMode(damper_1d, INPUT_PULLUP);
	pinMode(damper_2a, INPUT_PULLUP);
	pinMode(damper_2b, INPUT_PULLUP);
	pinMode(damper_2c, INPUT_PULLUP);
	pinMode(damper_2d, INPUT_PULLUP);      //52  

	pinMode(start_led, OUTPUT);
	pinMode(ok_led, OUTPUT);
	pinMode(ng_led, OUTPUT);               //12
	pinMode(BUZZER, OUTPUT);               //13
	pinMode(discharge, OUTPUT);            //9

	pinMode(vol_1, INPUT);                  //MAX 25V 전압측정
	pinMode(vol_2, INPUT);

	pinMode(opt_1, INPUT);
	pinMode(opt_2, INPUT);
	pinMode(opt_3, INPUT);
	pinMode(opt_4, INPUT);

	//---------------------------검사시작 설정 ------------------------------------------------
	lcd.init();           // LCD 초기화
	lcd.backlight();
	lcd.clear();
	load_led_all_off();   // LED ALL OFF
	mp3_play(4);         // FT 검사 시작시 

}

//----------------------------메인 함수 시작-------------------------------------------------

void loop()
{
	// Serial.print("count_standby_2 : ");
	// Serial.println(count_standby_2);
	// delay(400);
	// Serial.print("opt_2 : ");
	// Serial.println(opt_value_2);
	// delay(400);
	// Serial.print("low3 : ");
	// Serial.println(opt_value1); 
	float temp_0;
	float temp_1;
	float temp_2;
	float temp_3;

	int input_vol_a0 = analogRead(A0);
	int input_vol_a1 = analogRead(A1);
	int input_vol_a2 = analogRead(A2);
	int input_vol_a3 = analogRead(A3);

	temp_0 = input_vol_a0 / 4.092;       // 아날로그 값
	temp_1 = input_vol_a1 / 4.092;
	temp_2 = input_vol_a2 / 4.092;
	temp_3 = input_vol_a3 / 4.092;

	cal_vol_a0 = temp_0 / 10;         // 계산전압 MAX 25V
	cal_vol_a1 = temp_1 / 10;
	cal_vol_a2 = temp_2 / 10;
	cal_vol_a2 = temp_3 / 10;

	int opt_value_1 = analogRead(opt_1);
	int opt_value_2 = analogRead(opt_2);
	int opt_value_3 = analogRead(opt_3);
	int opt_value_4 = analogRead(opt_4);

	led_on_off();
	power_start_stop();

	switch (vs_Test_order)
	{
	case 0:   // 테스트 제품군 선택
		if (digitalRead(start_btn) == LOW) {
			Load_on_count();
			if (load_on > 10) { // 10회 이상 ON 시
				vs_option_order = 0;
				vs_Test_order = 1;
				test_start_stop = 1;
			}
		}
		else
		{
			if (opt_value_1 < 300)
			{
				lcd.setCursor(0, 0);      lcd.print(F("D21_P55_2021   TEST "));       // 9 AC LOAD                   SW1 ON
				opt_model = 1;
			}
			else if (opt_value_2 < 300)
			{
				lcd.setCursor(0, 0);       lcd.print(F("D22_P55_2022   TEST "));       // 8AC 1DC LOAD               SW2 ON
				opt_model = 2;
			}
			else if (opt_value_3 < 300)
			{
				lcd.setCursor(0, 0);       lcd.print(F("D24_3ROOM_2024 TEST "));       // 4AC LOAD                   SW3 ON   
				opt_model = 3;
			}
			else if (opt_value_4 < 300) {
				lcd.setCursor(0, 0);        lcd.print(F("J19_D21_2021   TEST "));      //  3ROOM                     SW4 ON
				opt_model = 4;
			}
			else
			{
				lcd.setCursor(0, 0);        lcd.print(F("CONTINUE       TEST "));
				opt_model = 5;
			}
			lcd.setCursor(0, 1);        lcd.print(F("   2023.08.14 V1.1  "));
			lcd.setCursor(0, 2);        lcd.print(F("WINIA-ELEC. M-FACT  "));
			lcd.setCursor(0, 3);        lcd.print(F("Made by PJC-JHG-CSH "));
			test_start_stop = 0;
			digitalWrite(start_led, LOW);
			digitalWrite(ng_led, LOW);
			digitalWrite(power, LOW);
			load_led_all_off();
		}
		break;

	case 1:   // 테스트 시작 설정
		switch (vs_option_order)
		{
		case 0:
			count_standby_1 = 0;
			count_standby_3 = 0;
			count_step_on = 0;
			count_siso = 0;
			load_on = 0;
			test_start_stop = 1;   // test 시작 flag            
			digitalWrite(start_led, HIGH);
			digitalWrite(power, HIGH);
			vs_option_order = 1;
			lcd.clear();
			break;

		case 1: // 테스트 제품군별 로직 분기
			if (opt_model == 1 || opt_model == 2) {   // D55 
				vs_Test_order = 3;
			}
			else {
				vs_option_order = 2;
			}
			break;

		case 2:
			if (opt_model == 3) {                     // D24 3ROOM
				vs_Test_order = 200;
			}
			else {
				vs_option_order = 3;
			}
		case 3:
			if (opt_model == 4 || opt_model == 5) {    // J19
				vs_Test_order = 100;
			}
			else {
				vs_option_order = 0;
			}
			break;
		}
		break;

		// 미사용
	case 2:

		break;

		
	case 3:  // LOAD1 RELAY 동작 확인
		Standby_count();
		if (digitalRead(Load1_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {        // 10회 이상 ON 
				lcd.setCursor(0, 0);       lcd.print(F("1__ON"));
				tone(BUZZER, NOTE_C5, 100);
				count_standby = 0;
				load_on = 0;
				relay1_input = 1;
				vs_Test_order = 4;
			}
		}
		else {
			lcd.setCursor(0, 0);          lcd.print(F("1_OFF"));
			lcd.noCursor();
			lcd.noBlink();
			if (count_standby >= 400) {  // 400회 이상 시도
				relay1_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 51;      // 1번째 AC 부하 신호 불검출 시
			}
		}
		break;


		
	case 4: // LOAD2 RELAY 동작 확인
		Standby_count();
		if (digitalRead(Load2_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				lcd.setCursor(6, 0);     lcd.print(F("2__ON"));
				tone(BUZZER, NOTE_D5, 100);
				relay2_input = 1;
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 5;
			}
		}
		else {
			lcd.setCursor(6, 0);        lcd.print(F("2_OFF"));
			if (count_standby >= 200) {
				relay2_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 52;      // 2번째 AC 부하 신호 불검출 시 
			}
		}
		break;

		
	case 5: // LOAD2&3 RELAY 동시동작 확인
		if (digitalRead(Load2_relay) == LOW && digitalRead(Load3_relay) == LOW) {
			Load_on_count();
			if (load_on >= 15) {
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 43;
			}
		}
		else {
			count_standby = 0;
			load_on = 0;
			vs_Test_order = 6;
		}
		break;

		
	case 6:  // LOAD3 RELAY 동작 확인
		Standby_count();
		if (digitalRead(Load3_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				lcd.setCursor(12, 0);        lcd.print(F("3__ON"));
				tone(BUZZER, NOTE_E5, 100);
				relay3_input = 1;
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 7;
			}
		}
		else {
			lcd.setCursor(12, 0);      lcd.print(F("3_OFF"));
			if (count_standby >= 200) {
				relay3_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 53;              //   3번째 AC 부하 신호 불검출 시 
			}
		}
		break;

		
	case 7: // LOAD3&4 RELAY 동시동작 확인
		if (digitalRead(Load3_relay) == LOW && digitalRead(Load4_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 44;
			}
		}
		else {
			load_on = 0;
			vs_Test_order = 8;
		}
		break;

		
	case 8:  // LOAD4 RELAY 동작 확인
		Standby_count();
		if (digitalRead(Load4_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				lcd.setCursor(0, 1);      lcd.print(F("4__ON"));
				tone(BUZZER, NOTE_F5, 100);
				relay4_input = 1;
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 9;
			}
		}
		else {
			lcd.setCursor(0, 1);          lcd.print(F("4_OFF"));
			if (count_standby >= 200) {
				relay4_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 54;     // 4번째 AC 부하 신호 불검출 시 
			}
		}
		break;

		
	case 9: // LOAD4&5 RELAY 동시동작 확인
		if (digitalRead(Load4_relay) == LOW && digitalRead(Load5_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 45;
			}
		}
		else {
			load_on = 0;
			vs_Test_order = 10;
		}
		break;

		
	case 10:  // LOAD5 RELAY 동작 확인
		Standby_count();
		if (digitalRead(Load5_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				lcd.setCursor(6, 1);      lcd.print(F("5__ON"));
				tone(BUZZER, NOTE_G5, 100);
				relay5_input = 1;
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 11;
			}
		}
		else
		{
			lcd.setCursor(6, 1);         lcd.print(F("5_OFF"));
			if (count_standby >= 200) {
				relay5_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 55;                //   5번째 AC 부하 신호 불검출 시 
			}
		}
		break;

		
	case 11: // LOAD5&6 RELAY 동시동작 확인
		if (digitalRead(Load5_relay) == LOW && digitalRead(Load6_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 46;
			}
		}
		else {
			load_on = 0;
			vs_Test_order = 12;
		}
		break;


		
	case 12: // LOAD6 RELAY 동작 확인
		Standby_count();
		if (digitalRead(Load6_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				lcd.setCursor(12, 1);      lcd.print(F("6__ON"));
				tone(BUZZER, NOTE_A5, 100);
				relay6_input = 1;
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 13;
			}
		}
		else {
			lcd.setCursor(12, 1);          lcd.print(F("6_OFF"));
			if (count_standby >= 200)
			{
				relay6_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 56;          //   6번째 AC 부하 신호 불검출 시 
			}
		}
		break;

		
	case 13: // LOAD6&7 RELAY 동시동작 확인
		if (digitalRead(Load6_relay) == LOW && digitalRead(Load7_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 47;
			}
		}
		else {
			load_on = 0;
			vs_Test_order = 14;
		}
		break;

		
	case 14: // LOAD7 RELAY 동작 확인
		if (opt_model == 1)
		{
			Standby_count();
			if (digitalRead(Load7_relay) == LOW) {
				Load_on_count();
				if (load_on >= 10) {
					lcd.setCursor(0, 2);       lcd.print(F("7__ON"));
					tone(BUZZER, NOTE_B5, 100);
					relay7_input = 1;
					count_standby = 0;
					load_on = 0;
					vs_Test_order = 16;
				}
			}
			else
			{
				lcd.setCursor(0, 2);          lcd.print(F("7_OFF"));
				if (count_standby >= 200) {
					count_standby = 0;
					load_on = 0;
					mp3_play(2);
					delay(10);
					vs_Test_order = 57;     //   9번째 AC 부하 신호 불검출 시 
				}
			}
		}
		else
			vs_Test_order = 15;

		break;

		
	case 15:  // LOAD8 RELAY 동작 확인
		if (opt_model == 2) {
			Standby_count();
			if (digitalRead(Load_doorheat) == LOW) {
				Load_on_count();
				if (load_on >= 10)
				{
					lcd.setCursor(0, 2);       lcd.print(F("D__ON"));
					tone(BUZZER, NOTE_B5, 100);
					count_standby = 0;
					load_on = 0;
					vs_Test_order = 16;
				}
			}
			else {
				lcd.setCursor(0, 2);          lcd.print(F("D_OFF"));
				if (count_standby >= 200) {
					count_standby = 0;
					load_on = 0;
					mp3_play(2);
					delay(10);
					vs_Test_order = 60;
				}
			}
		}
		else
			vs_Test_order = 16;

		break;


		
	case 16: // LOAD7&8 RELAY 동시동작 확인
		if (digitalRead(Load7_relay) == LOW && digitalRead(Load8_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 48;
			}
		}
		else {
			load_on = 0;
			vs_Test_order = 17;
		}
		break;

		
	case 17:  // LOAD8 RELAY 동작 확인
		Standby_count();
		if (digitalRead(Load8_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				lcd.setCursor(6, 2);      lcd.print(F("8__ON"));
				tone(BUZZER, NOTE_C6, 100);
				count_standby = 0;
				load_on = 0;

				vs_Test_order = 18;
			}
		}
		else
		{
			lcd.setCursor(6, 2);         lcd.print(F("8_OFF"));
			if (count_standby >= 200) {
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 59;     //   9번째 AC 부하 신호 불검출 시  
			}
		}
		break;

		
	case 18: // Load_comp 동작 확인
		if (count_comp >= 2 && digitalRead(Load_comp) == LOW) {
			lcd.setCursor(0, 3);     lcd.print(F("Comp_O"));
			count_comp = 0;
			vs_Test_order = 21;
		}
		else { // 3초 이상 동작 안했을 경우
			lcd.setCursor(0, 3);     lcd.print(F("Comp_X"));
			mp3_play(2);
			delay(10);
			count_comp = 0;
			vs_Test_order = 61;
		}
		break;

	case 19:
		break;

	case 20:
		break;

	case 21: // step_a 동작 확인
		Standby_count();
		if (digitalRead(step_a) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				lcd.setCursor(6, 3);       lcd.print("A");
				tone(BUZZER, NOTE_C6, 100);
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 22;
			}
		}
		else
		{
			lcd.setCursor(6, 3);          lcd.print(" ");
			if (count_standby >= 200) {
				relay8_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 62;
			}
		}
		break;

	case 22: // step_b 동작 확인
		Standby_count();
		if (digitalRead(step_b) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				lcd.setCursor(7, 3);       lcd.print("B");
				tone(BUZZER, NOTE_D6, 100);
				relay8_input = 1;
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 23;
			}
		}
		else
		{
			lcd.setCursor(7, 3);          lcd.print(" ");
			if (count_standby >= 200) {
				relay8_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 62;
			}
		}
		break;

	case 23: // step_c 동작 확인
		Standby_count();
		if (digitalRead(step_c) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				lcd.setCursor(8, 3);       lcd.print("C");
				tone(BUZZER, NOTE_E6, 100);
				relay8_input = 1;
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 24;
			}
		}
		else {
			lcd.setCursor(8, 3);         lcd.print(" ");
			if (count_standby >= 200) {
				relay8_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 62;
			}
		}
		break;

	case 24: // step_d 동작 확인
		Standby_count();
		if (digitalRead(step_d) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				lcd.setCursor(9, 3);      lcd.print("D");
				tone(BUZZER, NOTE_F6, 100);
				relay8_input = 1;
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 25;
			}
		}
		else {
			lcd.setCursor(9, 3);       lcd.print(" ");
			if (count_standby >= 200) {
				relay8_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 62;
			}
		}
		break;

	case 25:  // damper_1a & damper_1d 동작 확인
		Standby_count();
		if (digitalRead(damper_1a) == LOW && digitalRead(damper_1d) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				lcd.setCursor(11, 3);lcd.print("A");
				lcd.setCursor(14, 3);lcd.print("D");
				tone(BUZZER, NOTE_G6, 100);
				relay8_input = 1;
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 26;
			}
		}
		else {
			lcd.setCursor(11, 3);lcd.print(" ");
			lcd.setCursor(14, 3);lcd.print(" ");
			if (count_standby >= 200) {
				relay8_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 63;
			}
		}
		break;

	case 26: // damper_1b & damper_1c 동작 확인
		Standby_count();
		if (digitalRead(damper_1b) == LOW && digitalRead(damper_1c) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				lcd.setCursor(12, 3);         lcd.print("B");
				lcd.setCursor(13, 3);         lcd.print("C");
				tone(BUZZER, NOTE_A6, 100);
				relay8_input = 1;
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 27;
			}
		}
		else {
			lcd.setCursor(12, 3);           lcd.print(" ");
			lcd.setCursor(13, 3);           lcd.print(" ");
			if (count_standby >= 200) {
				relay8_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 63;
			}
		}
		break;

	case 27:  // damper_2a & damper_2d 동작 확인
		Standby_count();
		if (digitalRead(damper_2a) == LOW && digitalRead(damper_2d) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				lcd.setCursor(16, 3);        lcd.print("A");
				lcd.setCursor(19, 3);        lcd.print("D");
				tone(BUZZER, NOTE_B6, 100);
				relay8_input = 1;
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 28;
			}
		}
		else {
			lcd.setCursor(16, 3);          lcd.print(" ");
			lcd.setCursor(18, 3);          lcd.print(" ");
			if (count_standby >= 200) {
				relay8_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 64;
			}
		}
		break;

	case 28:
		Standby_count();
		if (digitalRead(damper_2b) == LOW && digitalRead(damper_2c) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				lcd.setCursor(17, 3);        lcd.print("B");
				lcd.setCursor(18, 3);        lcd.print("C");
				tone(BUZZER, NOTE_C7, 100);
				relay8_input = 1;
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 37;
			}
		}
		else {
			lcd.setCursor(17, 3);          lcd.print(" ");
			lcd.setCursor(18, 3);          lcd.print(" ");
			if (count_standby >= 200) {
				relay8_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 64;
			}
		}
		break;

	case 29:
		break;

	case 30:
		break;

	case 31:
		break;

	case 32:
		break;

	case 33:
		break;

	case 34:
		break;

		//--------------------DC입력 확인 구간-------------------------------  
	case 35:
		break;

		//------------------- DC입력 확인 구간------------------------------ 
	case 36:
		lcd.setCursor(14, 0);      lcd.print(cal_vol_a0);        //5v 전압을 lcd로 출력
		lcd.setCursor(14, 1);      lcd.print(cal_vol_a2);        //12v 전압을 lcd로 출력
		delay(1000);
		vs_Test_order = 37;
		break;

		//---------------------테스트 완료 후 이동구간----------------------------
	case 37:
		delay(200);
		load_led_11_off();
		count_standby = 0;
		count_standby_3 = 0;
		vs_SSub_order = 0;
		lcd.clear();
		vs_Test_order = 38;
		break;
		//--------------------------------------------------------------------------------------------------------
	case 38:                              // 테스트 시간이 짧을 경우 늘리는 설정 초로 설정
		switch (vs_SSub_order)
		{
		case 0:
			if (opt_model == 1) {          // D55 D21

				count_standby_1 = 9;
				vs_SSub_order = 1;
			}
			else if (opt_model == 2) {      // D55 D22
				count_standby_1 = 9;
				vs_SSub_order = 2;
			}
			else if (opt_model == 3) {     // D23 3ROOM
				count_standby_1 = 1;
				vs_SSub_order = 3;
			}
			else if (opt_model == 4) {     // J19_D22      
				count_standby_1 = 9;
				vs_SSub_order = 4;
			}
			else {
				count_standby_1 = 9;    //    
				vs_SSub_order = 5;
			}
			break;

		case 1:                         // Opt1 model =  9 AC LOAD  -COMP 동작
			drcrease_count_1s();
			if (count_standby_1 == 0) {
				count_standby_1 = 0;
				vs_Test_order = 39;
			}
			else {
				lcd.setCursor(0, 0);       lcd.print(F("AC&DC LOAD OK       "));
				lcd.setCursor(0, 1);       lcd.print(F("D21_P55_2021        "));
				lcd.setCursor(0, 2);       lcd.print(F("CHECK ROTATE FAN    "));
				lcd.setCursor(19, 3);      lcd.print(count_standby_1);
			}
			break;

		case 2:
			drcrease_count_1s();             // Opt2 model = 8AC 1DC LOAD -COMP 동작
			if (count_standby_1 == 0) {
				count_standby_1 = 0;
				vs_Test_order = 39;
			}
			else {
				lcd.setCursor(0, 0);       lcd.print(F("AC&DC LOAD OK       "));
				lcd.setCursor(0, 1);       lcd.print(F("D22_P55_2022        "));
				lcd.setCursor(0, 2);       lcd.print(F("CHECK ROTATE FAN    "));
				lcd.setCursor(19, 3);      lcd.print(count_standby_1);
			}
			break;

		case 3:
			drcrease_count_1s();            // Opt3 model = JI 1DOOR 
			if (count_standby_1 == 0) {
				count_standby_1 = 0;
				vs_Test_order = 39;
			}
			else {
				lcd.setCursor(0, 0);        lcd.print(F("AC&DC LOAD OK       "));
				lcd.setCursor(0, 1);        lcd.print(F("J19_D21_2021        "));
				lcd.setCursor(0, 2);        lcd.print(F("CHECK ROTATE FAN    "));
				lcd.setCursor(19, 3);       lcd.print(count_standby_1);
			}
			break;

		case 4:
			drcrease_count_1s();             // Opt4 model = JS (4) 
			if (count_standby_1 == 0) {
				count_standby_1 = 0;
				vs_Test_order = 39;
			}
			else {
				lcd.setCursor(0, 0);         lcd.print(F("AC&DC LOAD OK       "));
				lcd.setCursor(0, 1);         lcd.print(F("J19_D22_2022        "));
				lcd.setCursor(0, 2);         lcd.print(F("CHECK ROTATE FAN    "));
				lcd.setCursor(19, 3);        lcd.print(count_standby_1);
			}
			break;

		case 5:
			drcrease_count_1s();             // Opt5 model = JS (3)  
			if (count_standby_1 == 0) {
				count_standby_1 = 0;
				vs_Test_order = 39;
			}
			else {
				lcd.setCursor(0, 0);           lcd.print(F("AC&DC LOAD OK       "));
				lcd.setCursor(0, 1);           lcd.print(F("NO-USE              "));
				lcd.setCursor(0, 2);           lcd.print(F("CHECK ROTATE FAN    "));
				lcd.setCursor(19, 3);          lcd.print(count_standby_1);
			}
			break;
		}
		break;
		//-------------------------------------------------------------------------------------------------      
	case 39:
		vs_SSub_order = 0;
		load_led_all_off();
		delay(10);
		mp3_play(1);
		delay(10);
		count_standby = 0;
		vs_Test_order = 40;
		break;

	case 40:
		if (opt_model == 1) {
			lcd.setCursor(0, 1);     lcd.print(F("D21_P55_21_TEST   OK"));
			vs_Test_order = 69;
		}
		else if (opt_model == 2) {
			lcd.setCursor(0, 1);     lcd.print(F("D22_P55_2022  TEST  "));
			vs_Test_order = 69;
		}
		else if (opt_model == 3) {
			lcd.setCursor(0, 1);     lcd.print(F("D21_J19_2021_TEST OK"));
			vs_Test_order = 69;
		}
		else if (opt_model == 4) {
			lcd.setCursor(0, 1);     lcd.print(F("D22_J19_2022_TEST OK"));
			vs_Test_order = 69;
		}
		else {
			lcd.setCursor(0, 1);     lcd.print(F("NO_USE     TEST OK "));
			vs_Test_order = 69;
		}
		break;

	case 41:
		break;

	case 42:
		time_siso();
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("LOAD1&2 SHORT       "));
		lcd.setCursor(0, 1);       lcd.print(F("SHORT CHECK PLZ     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		digitalWrite(ng_led, HIGH);
		tone(BUZZER, ton_ng, 500);
		if (flag == 1) {
			digitalWrite(led_1st, !(digitalRead(led_1st)));
			digitalWrite(led_2nd, !(digitalRead(led_2nd)));
			flag = 0;
		}
		if (count_siso == 6) { //테스트종료후 부하 LED 점등 상태 수정
			load_led_11_off();
			count_siso = 0;
			vs_Test_order = 70;
		}
		break;

	case 43:
		time_siso();
		test_start_stop = 0;
		cycle_complete = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("LOAD2&3 SHORT       "));
		lcd.setCursor(0, 1);       lcd.print(F("SHORT CHECK PLZ     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		digitalWrite(ng_led, HIGH);
		tone(BUZZER, ton_ng, 500);
		if (flag == 1) {
			digitalWrite(led_2nd, !(digitalRead(led_2nd)));
			digitalWrite(led_3rd, !(digitalRead(led_3rd)));
			flag = 0;
		}
		if (count_siso == 6) {
			load_led_11_off();
			count_siso = 0;
			vs_Test_order = 70; // 테스트종료후 부하 LED 점등 상태 수정
		}
		break;

	case 44:
		time_siso();
		test_start_stop = 0;
		cycle_complete = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("LOAD3&4 SHORT       "));
		lcd.setCursor(0, 1);       lcd.print(F("SHORT CHECK PLZ     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		digitalWrite(ng_led, HIGH);
		tone(BUZZER, ton_ng, 500);
		if (flag == 1) {
			digitalWrite(led_3rd, !(digitalRead(led_3rd)));
			digitalWrite(led_4th, !(digitalRead(led_4th)));
			flag = 0;
		}
		if (count_siso == 6) {
			load_led_11_off();
			count_siso = 0;
			vs_Test_order = 70; // 테스트종료후 부하 LED 점등 상태 수정
		}
		break;

	case 45:
		time_siso();
		test_start_stop = 0;
		cycle_complete = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("LOAD4&5 SHORT       "));
		lcd.setCursor(0, 1);       lcd.print(F("SHORT CHECK PLZ     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		digitalWrite(ng_led, HIGH);
		tone(BUZZER, ton_ng, 500);
		if (flag == 1) {
			digitalWrite(led_4th, !(digitalRead(led_4th)));
			digitalWrite(led_5th, !(digitalRead(led_5th)));
			flag = 0;
		}
		if (count_siso == 6) {
			load_led_11_off();
			count_siso = 0;
			vs_Test_order = 70;
		}
		break;

	case 46:
		time_siso();
		test_start_stop = 0;
		cycle_complete = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("LOAD5&6 SHORT       "));
		lcd.setCursor(0, 1);       lcd.print(F("SHORT CHECK PLZ     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		digitalWrite(ng_led, HIGH);
		tone(BUZZER, ton_ng, 500);
		if (flag == 1) {
			digitalWrite(led_5th, !(digitalRead(led_5th)));
			digitalWrite(led_6th, !(digitalRead(led_6th)));
			flag = 0;
		}
		if (count_siso == 6) {
			load_led_11_off();
			count_siso = 0;
			vs_Test_order = 70;
		}
		break;

	case 47:
		time_siso();
		test_start_stop = 0;
		cycle_complete = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("LOAD6&7 SHORT       "));
		lcd.setCursor(0, 1);       lcd.print(F("SHORT CHECK PLZ     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		digitalWrite(ng_led, HIGH);
		tone(BUZZER, ton_ng, 500);
		if (flag == 1) {
			digitalWrite(led_5th, !(digitalRead(led_6th)));
			digitalWrite(led_6th, !(digitalRead(led_7th)));
			flag = 0;
		}
		if (count_siso == 6) {
			load_led_11_off();
			count_siso = 0;
			vs_Test_order = 70;
		}
		break;

	case 48:
		time_siso();
		test_start_stop = 0;
		cycle_complete = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("LOAD7&8 SHORT       "));
		lcd.setCursor(0, 1);       lcd.print(F("SHORT CHECK PLZ     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		if (flag == 1) {
			digitalWrite(led_7th, !(digitalRead(led_7th)));
			digitalWrite(led_8th, !(digitalRead(led_8th)));
			flag = 0;
		}
		if (count_siso == 6) {
			load_led_11_off();
			count_siso = 0;
			vs_Test_order = 70;
		}
		break;

	case 49:
		time_siso();
		test_start_stop = 0;
		cycle_complete = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("LOAD8&9 SHORT       "));
		lcd.setCursor(0, 1);       lcd.print(F("SHORT CHECK PLZ     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		if (flag == 1) {
			digitalWrite(led_8th, !(digitalRead(led_8th)));
			digitalWrite(led_9th, !(digitalRead(led_9th)));
			flag = 0;
		}
		if (count_siso == 6) {
			load_led_11_off();
			count_siso = 0;
			vs_Test_order = 70;
		}
		break;

	case 50:

		break;

	case 51:
		time_siso();
		test_start_stop = 0;
		cycle_complete = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("1_RY_N.G            "));
		lcd.setCursor(0, 1);       lcd.print(F("PLZ_LOAD1_CHECK     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		if (flag == 1)        digitalWrite(led_1st, HIGH);
		else if (flag == 2)   digitalWrite(led_1st, LOW);
		else {
			flag = 0;
			if (count_siso >= 8)
			{
				load_led_11_off();
				count_siso = 0;
				vs_Test_order = 70;
			}
		}
		break;

	case 52:
		time_siso();
		cycle_complete = 0;
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("2_RY_N.G            "));
		lcd.setCursor(0, 1);       lcd.print(F("PLZ_LOAD2_CHECK     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		if (flag == 1)       digitalWrite(led_2nd, HIGH);
		else if (flag == 2)  digitalWrite(led_2nd, LOW);
		else {
			flag = 0;
			if (count_siso >= 8) {
				load_led_11_off();
				count_siso = 0;
				vs_Test_order = 70;
			}
		}
		break;

	case 53:
		time_siso();
		cycle_complete = 0;
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("3_RY_N.G            "));
		lcd.setCursor(0, 1);       lcd.print(F("PLZ_LOAD3_CHECK     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		if (flag == 1)          digitalWrite(led_3rd, HIGH);
		else if (flag == 2)     digitalWrite(led_3rd, LOW);
		else {
			flag = 0;
			if (count_siso >= 8) {
				load_led_11_off();
				count_siso = 0;
				vs_Test_order = 70;
			}
		}
		break;

	case 54:
		time_siso();
		cycle_complete = 0;
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("4_RY_N.G            "));
		lcd.setCursor(0, 1);       lcd.print(F("PLZ_LOAD4_CHECK     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		if (flag == 1)       digitalWrite(led_4th, HIGH);
		else if (flag == 2)  digitalWrite(led_4th, LOW);
		else {
			flag = 0;
			if (count_siso >= 8) {
				load_led_11_off();
				count_siso = 0;
				vs_Test_order = 70;
			}
		}
		break;

	case 55:
		time_siso();
		cycle_complete = 0;
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("5_RY_N.G            "));
		lcd.setCursor(0, 1);       lcd.print(F("PLZ_LOAD5_CHECK     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		if (flag == 1)         digitalWrite(led_5th, HIGH);
		else if (flag == 2)    digitalWrite(led_5th, LOW);
		else {
			flag = 0;
			if (count_siso >= 8) {
				load_led_11_off();
				count_siso = 0;
				vs_Test_order = 70;
			}
		}
		break;

	case 56:
		time_siso();
		test_start_stop = 0;
		cycle_complete = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("6_RY_N.G            "));
		lcd.setCursor(0, 1);       lcd.print(F("PLZ_LOAD6_CHECK     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		if (flag == 1)        digitalWrite(led_6th, HIGH);
		else if (flag == 2)   digitalWrite(led_6th, LOW);
		else {
			flag = 0;
			if (count_siso >= 8) {
				load_led_11_off();
				count_siso = 0;
				vs_Test_order = 70;
			}
		}
		break;

	case 57:
		time_siso();
		cycle_complete = 0;
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("7_RY_N.G            "));
		lcd.setCursor(0, 1);       lcd.print(F("PLZ_LOAD7_CHECK     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		if (flag == 1)  digitalWrite(led_7th, HIGH);
		else if (flag == 2)    digitalWrite(led_7th, LOW);
		else {
			flag = 0;
			if (count_siso >= 8) {
				load_led_11_off();
				count_siso = 0;
				vs_Test_order = 70;
			}
		}
		break;

	case 58:
		time_siso();
		cycle_complete = 0;
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("D_heater_N.G        "));
		lcd.setCursor(0, 1);       lcd.print(F("PLZ_DHEATER_CHECK   "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		if (flag == 1)       digitalWrite(led_8th, HIGH);
		else if (flag == 2)  digitalWrite(led_8th, LOW);
		else {
			flag = 0;
			if (count_siso >= 8) {
				load_led_11_off();
				count_siso = 0;
				vs_Test_order = 70;
			}
		}
		break;

	case 59:
		time_siso();
		cycle_complete = 0;
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("8_RY_N.G            "));
		lcd.setCursor(0, 1);       lcd.print(F("PLZ_LOAD8_CHECK     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		if (flag == 1)       digitalWrite(led_8th, HIGH);
		else if (flag == 2)  digitalWrite(led_8th, LOW);
		else {
			flag = 0;
			if (count_siso >= 8) {
				load_led_11_off();
				count_siso = 0;
				vs_Test_order = 70;
			}
		}
		break;

	case 60:
		time_siso();
		cycle_complete = 0;
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("DOOR_HEATER N.G     "));
		lcd.setCursor(0, 1);       lcd.print(F("PLZ_DOORHEATER_CHECK"));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		if (flag == 1)  digitalWrite(led_7th, HIGH);
		else if (flag == 2)    digitalWrite(led_7th, LOW);
		else {
			flag = 0;
			if (count_siso >= 8) {
				load_led_11_off();
				count_siso = 0;
				vs_Test_order = 25;
			}
		}
		break;

	case 61:
		time_siso();
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("COMP_WORKING N.G    "));
		lcd.setCursor(0, 1);       lcd.print(F("COMP_WORKING_CHECK  "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		if (flag == 1)  digitalWrite(led_comp, HIGH);
		else if (flag == 2)    digitalWrite(led_comp, LOW);
		else {
			flag = 0;
			if (count_siso >= 8) {
				load_led_11_off();
				count_siso = 0;
				vs_Test_order = 25;
			}
		}

		break;

	case 62:
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("STEP VALVE N.G      "));
		lcd.setCursor(0, 1);       lcd.print(F("STEP VALVE CHECK    "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		vs_Test_order = 70;
		break;

	case 63:
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("DAMPER_1   N.G      "));
		lcd.setCursor(0, 1);       lcd.print(F("DAMPER_1 _CHECK     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		vs_Test_order = 70;
		break;

	case 64:
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("DAMPER_2   N.G      "));
		lcd.setCursor(0, 1);       lcd.print(F("DAMPER_2 _CHECK     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		vs_Test_order = 70;
		break;

	case 65:

		break;

	case 66:
		break;

	case 67:
		break;

	case 68:
		break;

	case 69:
		load_led_11_off();                // 2021-4-02 테스트종료후 부하 LED 점등 상태 수정
		digitalWrite(ok_led, HIGH);
		digitalWrite(power, LOW);
		delay(200);
		digitalWrite(start_led, LOW);
		digitalWrite(discharge, LOW);
		test_start_stop = 0;
		count_standby_1 = 5;
		count_step_on = 0;
		vs_Test_order = 71;
		break;

	case 70: // 테스트종료후 부하 LED 점등 상태 수정
		load_led_11_off();
		digitalWrite(ng_led, HIGH);
		digitalWrite(power, LOW);
		delay(200);
		digitalWrite(start_led, LOW);
		digitalWrite(discharge, LOW);
		test_start_stop = 0;
		count_standby = 0;
		cycle_complete = 0;
		count_step_on = 0;
		count_standby_1 = 5;
		vs_Test_order = 71;
		break;

	case 71:
		drcrease_count_1s();
		if (test_start_stop == 0) {
			if (count_standby_1 == 0) {
				load_on = 0;
				count_standby_1 = 0;
				count_step_on = 0;
				vs_Test_order = 72;

			}
			else {
				digitalWrite(power, LOW);
				test_start_stop = 0;
				load_on = 0;
				lcd.noBlink();
				lcd.setCursor(0, 2);           lcd.print(F("                    "));
				lcd.setCursor(0, 3);           lcd.print(F("5s_Standby        "));
				lcd.setCursor(19, 3);          lcd.print(count_standby_1);
			}
		}
		else {
			lcd.setCursor(0, 0);      lcd.print(F("ERROR!!ERROR!!ERRO!!"));
			lcd.setCursor(0, 1);      lcd.print(F("PUSH  STOP BUTTON !!"));
			lcd.setCursor(0, 2);      lcd.print(F("PUSH  STOP BUTTON !!"));
			lcd.setCursor(0, 3);      lcd.print(F("PUSH  STOP BUTTON !!"));
		}
		break;

	case 72:
		load_led_all_off();
		lcd.clear();
		vs_Test_order = 0;
		break;

	case 73:
		if (opt_value_1 == 0)               opt_model = 1;
		else if (opt_value_2 == 0)          opt_model = 2;
		else if (opt_value_3 == 0)          opt_model = 3;
		else if (opt_value_4 == 0)          opt_model = 4;
		else {
			opt_model = 5;
		}
		load_led_all_off();
		lcd.clear();
		vs_Test_order = 1;
		break;


		//--------------------JS (4) LOAD TEST START -------------------------------------
	case 100:
		lcd.setCursor(0, 0);   lcd.print(F("ONLY CONTINUE  TEST "));
		lcd.setCursor(0, 1);   lcd.print(F("   2023.08.14 V1.1  "));
		lcd.setCursor(0, 2);   lcd.print(F("WINIA-ELEC. M-FACT  "));
		lcd.setCursor(0, 3);   lcd.print(F("Made by PJC-JHG-CSH "));
		break;



		//--------------------JS (4) LOAD TEST START -------------------------------------
	case 101:
		Standby_count();
		if (digitalRead(Load1_relay) == LOW)
		{
			Load_on_count();
			if (load_on >= 10)
			{
				lcd.setCursor(0, 0);         lcd.print(F("1__ON"));
				tone(BUZZER, NOTE_C5, 100);
				count_standby = 0;
				load_on = 0;
				relay1_input = 1;
				vs_Test_order = 102;
			}
		}
		else {
			lcd.setCursor(0, 0);            lcd.print(F("1_OFF"));
			if (count_standby >= 400) {
				relay1_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 116;         //   1번째 AC 부하 신호 불검출 시
			}
		}
		break;

		//---------------------LOAD1&2 RELAY 동시동작 확인 ------------------------------- 
	case 102:
		if (digitalRead(Load1_relay) == LOW && digitalRead(Load2_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 111;
			}
		}
		else {
			load_on = 0;
			count_standby = 0;
			vs_Test_order = 103;
		}
		break;

		//---------------------LOAD2 RELAY 동작 확인 ---------------------------------
	case 103:
		Standby_count();
		if (digitalRead(Load2_relay) == LOW)
		{
			Load_on_count();
			if (load_on >= 10)
			{
				lcd.setCursor(6, 0);              lcd.print(F("2__ON"));
				tone(BUZZER, NOTE_D5, 100);
				relay2_input = 1;
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 104;
			}
		}
		else {
			lcd.setCursor(6, 0);
			lcd.print("2_OFF");
			if (count_standby >= 300) {
				relay2_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 117;              //   2번째 AC 부하 신호 불검출 시 
			}
		}
		break;

		//---------------------LOAD2&3 RELAY 동시동작 확인 ------------------------------- 
	case 104:
		if (digitalRead(Load2_relay) == LOW && digitalRead(Load3_relay) == LOW) {
			Load_on_count();
			if (load_on >= 15) {
				relay2_input = 1;
				relay3_input = 1;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 112;
			}
		}
		else {
			count_standby = 0;
			load_on = 0;
			vs_Test_order = 105;
		}
		break;

		//---------------------LOAD3 RELAY 동작 확인 ---------------------------------

	case 105:
		Standby_count();
		if (digitalRead(Load3_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				lcd.setCursor(13, 0);        lcd.print(F("3__ON"));
				tone(BUZZER, NOTE_E5, 100);
				relay3_input = 1;
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 106;
			}
		}
		else
		{
			lcd.setCursor(13, 0);           lcd.print(F("3_OFF"));
			if (count_standby >= 300) {
				relay3_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 118;              //   3번째 AC 부하 신호 불검출 시 
			}
		}
		break;

		//-------------------------LOAD3&4 RELAY 동시동작 확인 ----------------------------- 
	case 106:
		if (digitalRead(Load3_relay) == LOW && digitalRead(Load4_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 113;
			}
		}
		else {
			load_on = 0;
			vs_Test_order = 107;
		}
		break;

		//-------------------------LOAD4 RELAY 동작 확인 ----------------------------- 
	case 107:
		Standby_count();
		if (digitalRead(Load4_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				lcd.setCursor(0, 1);              lcd.print(F("4__ON"));
				tone(BUZZER, NOTE_F5, 100);
				relay4_input = 1;
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 108;
			}
		}
		else
		{
			lcd.setCursor(0, 1);
			lcd.print("4_OFF");
			if (count_standby >= 300) {
				relay4_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 119;     // 4번째 AC 부하 신호 불검출 시 
			}
		}
		break;

		//-------------------------LOAD5 door heater 동작 확인 ----------------------------- 
	case 108:
		break;


	case 111:
		time_siso();
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);      lcd.print(F("LOAD1&2 SHORT       "));
		lcd.setCursor(0, 1);      lcd.print(F("SHORT CHECK PLZ     "));
		lcd.setCursor(0, 2);     lcd.print(F("                    "));
		lcd.setCursor(0, 3);     lcd.print(F("                    "));
		digitalWrite(ng_led, HIGH);
		tone(BUZZER, ton_ng, 500);
		if (flag == 1) {
			digitalWrite(led_1st, !(digitalRead(led_1st)));
			digitalWrite(led_2nd, !(digitalRead(led_2nd)));
			flag = 0;
		}
		if (count_siso == 6) {
			load_led_11_off();
			count_siso = 0;
			vs_Test_order = 70;
		}
		break;

	case 112:
		time_siso();
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("LOAD2&3 SHORT       "));
		lcd.setCursor(0, 1);       lcd.print(F("SHORT CHECK PLZ     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		digitalWrite(ng_led, HIGH);
		tone(BUZZER, ton_ng, 500);
		if (flag == 1) {
			digitalWrite(led_2nd, !(digitalRead(led_2nd)));
			digitalWrite(led_3rd, !(digitalRead(led_3rd)));
			flag = 0;
		}
		if (count_siso == 6) {
			load_led_11_off();
			count_siso = 0;
			vs_Test_order = 70;
		}
		break;

	case 113:
		time_siso();
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("LOAD3&4 SHORT       "));
		lcd.setCursor(0, 1);       lcd.print(F("SHORT CHECK PLZ     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		digitalWrite(ng_led, HIGH);
		tone(BUZZER, ton_ng, 500);
		if (flag == 1) {
			digitalWrite(led_3rd, !(digitalRead(led_3rd)));
			digitalWrite(led_4th, !(digitalRead(led_4th)));
			flag = 0;
		}
		if (count_siso == 6) {
			load_led_11_off();
			count_siso = 0;
			vs_Test_order = 70;
		}
		break;

	case 114:
		break;

	case 115:
		break;


	case 116:
		time_siso();
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("R1_HTR_N.G          "));
		lcd.setCursor(0, 1);       lcd.print(F("PLZ_LOAD_1_CHECK    "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		if (flag == 1) {
			digitalWrite(led_1st, !(digitalRead(led_1st)));
			flag = 0;
		}
		if (count_siso == 6) {
			load_led_11_off();
			count_siso = 0;
			vs_Test_order = 70;
		}
		break;

	case 117:
		time_siso();
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("R3_HTR_N.G          "));
		lcd.setCursor(0, 1);       lcd.print(F("PLZ_LOAD_2_CHECK    "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		if (flag == 1) {
			digitalWrite(led_2nd, !(digitalRead(led_2nd)));
			flag = 0;
		}
		if (count_siso == 6) {
			load_led_11_off();
			count_siso = 0;
			vs_Test_order = 70;
		}
		break;

	case 118:
		time_siso();
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("R1_DRAIN_HTR_N.G    "));
		lcd.setCursor(0, 1);       lcd.print(F("PLZ_LOAD_3_CHECK    "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		if (flag == 1) {
			digitalWrite(led_3rd, !(digitalRead(led_3rd)));
			flag = 0;
		}
		if (count_siso == 6) {
			load_led_11_off();
			count_siso = 0;
			vs_Test_order = 70;
		}
		break;

	case 119:
		time_siso();
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("Pillar_HTR_N.G      "));
		lcd.setCursor(0, 1);       lcd.print(F("PLZ_LOAD4_CHECK     "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		if (flag == 1) {
			digitalWrite(led_4th, !(digitalRead(led_4th)));
			flag = 0;
		}
		if (count_siso == 6) {
			load_led_11_off();
			count_siso = 0;
			vs_Test_order = 70;
		}
		break;

	case 120:
		time_siso();
		test_start_stop = 0;
		digitalWrite(power, LOW);
		digitalWrite(start_led, LOW);
		lcd.setCursor(0, 0);       lcd.print(F("DOOR_HTR_NG         "));
		lcd.setCursor(0, 1);       lcd.print(F("PLZ_DOOR_HTR_CHECK  "));
		lcd.setCursor(0, 2);       lcd.print(F("                    "));
		lcd.setCursor(0, 3);       lcd.print(F("                    "));
		tone(BUZZER, ton_ng, 500);
		digitalWrite(ng_led, HIGH);
		if (flag == 1) {
			digitalWrite(led_5th, !(digitalRead(led_5th)));
			flag = 0;
		}
		if (count_siso == 6) {
			load_led_11_off();
			count_siso = 0;
			vs_Test_order = 70;
		}
		break;


		//-----------------------------D24 3ROOM 테스트 시작 --------------------------------------
	case 200:
		Standby_count();
		if (digitalRead(Load1_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				lcd.setCursor(0, 0);       lcd.print(F("1__ON"));
				tone(BUZZER, NOTE_C5, 100);
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 201;
			}
		}
		else {
			lcd.setCursor(0, 0);          lcd.print(F("1_OFF"));
			lcd.noCursor();
			lcd.noBlink();
			if (count_standby >= 500) {
				relay1_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 51;                 //   1번째 AC 부하 신호 불검출 시
			}
		}
		break;


		//---------------------LOAD2 RELAY 동작 확인 ---------------------------------
	case 201:
		Standby_count();
		if (digitalRead(Load2_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				lcd.setCursor(6, 0);     lcd.print(F("2__ON"));
				tone(BUZZER, NOTE_D5, 100);
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 202;
			}
		}
		else {
			lcd.setCursor(6, 0);      lcd.print(F("2_OFF"));
			if (count_standby >= 200) {
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 52;              //   2번째 AC 부하 신호 불검출 시 
			}
		}
		break;

		//---------------------LOAD2&3 RELAY 동시동작 확인 ------------------------------- 
	case 202:
		if (digitalRead(Load2_relay) == LOW && digitalRead(Load3_relay) == LOW) {
			Load_on_count();
			if (load_on >= 15) {
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 43;
			}
		}
		else {
			count_standby = 0;
			load_on = 0;
			vs_Test_order = 203;
		}
		break;

		//---------------------LOAD3 RELAY 동작 확인 ---------------------------------

	case 203:
		Standby_count();
		if (digitalRead(Load3_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10)
			{
				lcd.setCursor(12, 0);      lcd.print(F("3__ON"));
				tone(BUZZER, NOTE_E5, 100);
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 204;
			}
		}
		else {
			lcd.setCursor(12, 0);         lcd.print(F("3_OFF"));
			if (count_standby >= 200) {
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 53;              //   3번째 AC 부하 신호 불검출 시 
			}
		}
		break;

		//------------------------- LOAD3&4 RELAY 동시동작 확인 ----------------------------- 
	case 204:
		if (digitalRead(Load3_relay) == LOW && digitalRead(Load4_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 44;
			}
		}
		else {
			load_on = 0;
			vs_Test_order = 205;
		}
		break;

		//-------------------------LOAD4 RELAY 동작 확인 ----------------------------- 
	case 205:
		Standby_count();
		if (digitalRead(Load4_relay) == LOW) {
			Load_on_count();
			if (load_on >= 10) {
				lcd.setCursor(0, 1);        lcd.print(F("4__ON"));
				tone(BUZZER, NOTE_F5, 100);
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 206;
			}
		}
		else {
			lcd.setCursor(0, 1);
			lcd.print(F("4_OFF"));
			if (count_standby >= 200) {
				relay4_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 54;     // 4번째 AC 부하 신호 불검출 시 
			}
		}
		break;

		//----------------------------- comp 동작 확인 ------------------
	case 206:  // 3회째에만 comp 동작 확인
		if (cycle_complete == 2)      vs_Test_order = 207;
		else                          vs_Test_order = 210;
		break;

	case 207:
		if (count_comp >= 2 && digitalRead(Load_comp) == LOW) {
			lcd.setCursor(0, 2);          lcd.print(F("Comp_O"));
			count_comp = 0;
			vs_Test_order = 210;
		}
		else {
			lcd.setCursor(0, 2);          lcd.print(F("Comp_X"));
			mp3_play(2);
			delay(10);
			count_comp = 0;
			vs_Test_order = 61;       // 현재 시점에서 COMP 스위치가 3초 이상 동작 안했을 경우    
		}

		//-------------------------STEP VALVE  동작 확인 ----------------------------- 
	case 210:
		Standby_count();
		if (digitalRead(step_d) == LOW) {
			Load_on_count();
			if (load_on >= 5) {
				lcd.setCursor(0, 3);            lcd.print(F("A"));
				tone(BUZZER, NOTE_C6, 100);
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 211;
			}
		}
		else {
			lcd.setCursor(0, 3);              lcd.print(F(" "));
			if (count_standby >= 200) {
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 62;
			}
		}
		break;

	case 211:
		Standby_count();
		if (digitalRead(step_b) == LOW) {
			Load_on_count();
			if (load_on >= 5) {
				lcd.setCursor(1, 3);             lcd.print(F("B"));
				tone(BUZZER, NOTE_D6, 100);
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 212;
			}
		}
		else {
			lcd.setCursor(1, 3);                lcd.print(F(" "));
			if (count_standby >= 200) {
				relay8_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 62;
			}
		}
		break;

	case 212:
		Standby_count();
		if (digitalRead(step_c) == LOW) {
			Load_on_count();
			if (load_on >= 5) {
				lcd.setCursor(2, 3);           lcd.print(F("C"));
				tone(BUZZER, NOTE_E6, 100);
				count_standby = 0;
				load_on = 0;
				vs_Test_order = 213;
			}
		}
		else {
			lcd.setCursor(2, 3);              lcd.print(F(" "));
			if (count_standby >= 200) {
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 62;
			}
		}
		break;

	case 213:
		Standby_count();
		if (digitalRead(step_a) == LOW) {
			Load_on_count();
			if (load_on >= 5) {
				lcd.setCursor(3, 3);             lcd.print(F("D"));
				tone(BUZZER, NOTE_F6, 100);
				count_standby = 0;
				load_on = 0;
				cycle_complete++;
				if (cycle_complete <= 2)     vs_Test_order = 200; //D24 3ROOM 테스트 시작으로 복귀
				else {
					vs_Test_order = 214;
				}
			}
		}
		else {
			lcd.setCursor(3, 3);                lcd.print(F(" "));
			if (count_standby >= 200) {
				relay8_input = 0;
				count_standby = 0;
				load_on = 0;
				mp3_play(2);
				delay(10);
				vs_Test_order = 62;
			}
		}
		break;

		//------------------------COMP 동작 확인 -------------------------------------
	case 214:
		vs_Test_order = 37;
		cycle_complete = 0;
		break;
	}
}


/* 시간지연 millis(); 사용 방법
 unsigned long past = 0; // 과거 시간 저장 변수
 int flag = 0; // 과거 기준 시간 보다 500ms 이상 지날 경우를 판단하는 flag

void setup(){
	pinMode(13, OUTPUT);
}

void loop(){
	unsigned long now = millis(); // 현재 시간을 저장

	// 현재 시간이 과거 시간보다 500ms 지났을 때
	// 과거 시간에 현재 시간을 저장
	// 500ms 시간이 지낫음을 알려 주는 flag 를 1로 활성화
	if(now - past >= 500){
		past = now;
		flag = 1;
	}

	// flag 가 1인 경우
	// LED를 현재 상태 반전으로 출력
	// flag 를 0으로 초기화
	if(flag == 1){
		digitalWrite(13, !(digitalRead(13)));
		flag = 0;
	}

}
/*
   mp3_play (5);  //play "mp3/0005.mp3"   // mp3 폴더내의 0005 파일 재생
   void mp3_random_play ();

 */
