const int led1 = 19;
const int led2 = 17;
const int button1 = 24;
const int button2 = 19;

enum FsmState {Off = 0, Left, Right, Both};
FsmState state;

void setup() {
  
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  state = Off;
}

void loop() {
  switch (state) {
    case Off:
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
    case Right:
      digitalWrite(led1, LOW);
      digitalWrite(led2, HIGH);
    case Left:
      digitalWrite(led1, HIGH);
      digitalWrite(led2, LOW);
    case Both:
      digitalWrite(led1, HIGH);
      digitalWrite(led2, HIGH);
  }
  incr();
  decr();
  delay(500);
  
}

void incr() {
  if (digitalRead(button2)) {
    switch (state) {
      case Off:
        state = Right;
      case Right:
        state = Left;
      case Left:
        state = Both;
      case Both:
        state = Both;
    }
  }
}

void decr() {
  if (digitalRead(button1)) {
    switch (state) {
      case Off:
        state = Off;
      case Right:
        state = Off;
      case Left:
        state = Right;
      case Both:
        state = Left;
    }
  }
}
