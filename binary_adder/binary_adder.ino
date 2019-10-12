const int led1 = 10;
const int led2 = 9;
const int button1 = 14;
const int button2 = 15;

int cur_state;

void setup() {
  
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  cur_state = 1;
}

void loop() {
  switch (cur_state) {
    case 0:
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
      break;
    case 1:
      digitalWrite(led1, LOW);
      digitalWrite(led2, HIGH);
      break;
    case 2:
      digitalWrite(led1, HIGH);
      digitalWrite(led2, LOW);
      break;
    case 3:
      digitalWrite(led1, HIGH);
      digitalWrite(led2, HIGH);
      break;
  }
  incr();
  decr();

  
}

void incr() {
  if (digitalRead(button2) == 1) {
    if (cur_state < 3) {
      cur_state += 1;
      delay(500);
    }
  }
}

void decr() {
  if (digitalRead(button1) == 1) {
    if (cur_state > 0) {
      cur_state -= 1;
      delay(500);
    }
  }
}
