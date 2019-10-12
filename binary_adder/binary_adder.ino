const int led1 = 19;
const int led2 = 17;
const int button1 = 24;
const int button2 = 19;

int cur_state;

void setup() {
  
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  cur_state = 0;
}

void loop() {
  switch (cur_state) {
    case 0:
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
    case 1:
      digitalWrite(led1, LOW);
      digitalWrite(led2, HIGH);
    case 2:
      digitalWrite(led1, HIGH);
      digitalWrite(led2, LOW);
    case 3:
      digitalWrite(led1, HIGH);
      digitalWrite(led2, HIGH);
  }
  incr();
  decr();
  delay(500);
  
}

void incr() {
  if (digitalRead(button2)) {
    if (cur_state < 3) {
      cur_state += 1;
    }
  }
}

void decr() {
  if (digitalRead(button1)) {
    if (cur_state > 0) {
      cur_state -= 1;
    }
  }
}
