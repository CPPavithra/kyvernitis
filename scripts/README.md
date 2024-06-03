## Generate Shared Object for mother_interface.py
Move to scripts directory

```
git clone git@github.com:cmcqueen/cobs-c.git /tmp/cobs
gcc /tmp/cobs/cobs.c -shared -o libcobs.so.2.0.0
```

## Generate Shared Object for CRC calculation in mother_interface.py

```

gcc crc.c -shared -o crc.so.1.0.0
```

