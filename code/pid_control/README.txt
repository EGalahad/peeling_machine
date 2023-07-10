These are control code examples using pid control.

You can copy codes under `motors` directory to `src/main.cpp` to compile the code.

For example: to control two encoder motors to run at a target rpm

```bash
cp motors/two_encoder_motors.cpp src/main.cpp
pio run --target upload
```

