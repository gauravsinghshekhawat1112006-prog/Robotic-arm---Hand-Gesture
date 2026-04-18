  }
}

// ================= MAIN LOOP =================
void loop() {

  if (Serial.available()) {

    byte incoming = Serial.read();

    if (incoming == SYNC_BYTE) {

      while (Serial.available() < 6);

      byte a1 = Serial.read();
      byte a2 = Serial.read();
      byte step_angle = Serial.read();
      byte step_dir = Serial.read();
      byte g  = Serial.read();


      byte checksum = Serial.read();
      float angle= step_angle/100.0;
      if ((a1 + a2 + g + step_angle + step_dir) % 256 == checksum) {

        // --- Execute together ---
        servoBase.write(a1);
        servoElbow.write(a2);
        gripper.write(g);

        moveStepper(angle * 2.0, step_dir);
      }
    }
  }
}
