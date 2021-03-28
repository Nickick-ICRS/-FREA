# The byte to signal the end of a message
END_MSG_BYTE  = bytes([0xFF])
# The byte to signal that the next byte is for the left ear (0-100)
LE_SERVO_BYTE = bytes([0x01])
# The byte to signal that the next byte is for the right ear (0-100)
RE_SERVO_BYTE = bytes([0x02])
# The byte to signal that the next byte is for the mouth (0-100)
MOUTH_BYTE    = bytes([0x04])
# The byte to signal that the next byte is for the neck (0-100)
NECK_BYTE     = bytes([0x08])
# The byte to signal that the next 27 bytes are for the NeoPixels (HSV)
PIXELS_BYTE   = bytes([0x0F])

# The baud rate of the head
HEAD_BAUD     = 38400
