const int led1 = 10;
const int led2 = 9;
const int button1 = 14;
const int button2 = 15;

enum FsmState {None = 0, Left, Right, Both};
FsmState state;

void setup() {
  
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  state = Left;
}

void loop() {
  switch (state) {
    case None:
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
      break;
    case Right:
      digitalWrite(led1, LOW);
      digitalWrite(led2, HIGH);
      break;
    case Left:
      digitalWrite(led1, HIGH);
      digitalWrite(led2, LOW);
      break;
    case Both:
      digitalWrite(led1, HIGH);
      digitalWrite(led2, HIGH);
      break;
  }
  incr();
  decr();
  
  
}

void incr() {
  if (digitalRead(button2)) {
    switch (state) {
      case None:
        state = Right;
        break;
      case Right:
        state = Left;
        break;
      case Left:
        state = Both;
        break;
      case Both:
        state = Both;
        break;
    }
    delay(500);
  }
}

void decr() {
  if (digitalRead(button1)) {
    switch (state) {
      case None:
        state = None;
        break;
      case Right:
        state = None;
        break;
      case Left:
        state = Right;
        break;
      case Both:
        state = Left;
        break;
    }
    delay(500);
  }
}
