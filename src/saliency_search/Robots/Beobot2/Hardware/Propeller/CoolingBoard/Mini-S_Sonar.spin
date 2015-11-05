PUB getReading(PWM_PIN, TRIGGER_PIN) : distance  | start_time, end_time
  waitpne(|< PWM_PIN, |< PWM_PIN, 0)
  outa[TRIGGER_PIN] := 1

  waitpeq(|< PWM_PIN, |< PWM_PIN, 0)
  start_time := cnt

  waitpne(|< PWM_PIN, |< PWM_PIN, 0)
  end_time := cnt
  outa[TRIGGER_PIN] := 0
  distance := end_time - start_time
