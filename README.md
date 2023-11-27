# epdiy-usbDisplay
Use the epdiy ePaper controller as computer monitor over usb. This repo holds the firmware for the controller.

# Protocol
The picture gets sent to the controller with a serial stream, for the sake of simplicity over a virtual usb serial port (cdc acm). The protocol consists of packets of this form: `Command indicator | Command number | Comand dependent number of payload-bytes`. Payload bytes may hold information about positions on the display, namely row and column. It is noteworthy that the top left corner of the screen corresponds to row=0 and column=0. The row number increases by going down, and the column number increases by going right. With that, the packets work like this:

**Command indicator `0x80`:** This byte indicates that the next byte will be a command. So if the firmware taps into the stream at an arbitrary location, it just has to wait for the first command-indicator-byte to flow in and from there on understands what's going on. This also means that behind a `0x80` comes **always** a byte that tells the command, no exception to that.

**The individual commands:** Every command has a different number assigned, we will go through them here:
 - **`0x00` = Send a `0x80` as payload data:** Of course since we reserved `0x80` for indicating commands, we need to escape data that happens to be `0x80`. So any payload byte that happens to be `0x80` needs to be sent as the two bytes `0x80, 0x00`.
 - **`0x01` = Clear display:** Set all pixels to white, on both the pyhsical epaper as well as on the internal framebuffer. This is handy when the epaper state is unknown and also clears ghosting.
 - **`0x02` = Set origin position:** The origin position is a position on the display (column, row), and may be used by other commands, like drawing an image at a specific location. The column and row information are each a 16 bit unsigned integer value. The payload is 4 bytes:
    1. The upper 8 bits of the column
    2. The lower 8 bits of the column
    3. The upper 8 bits of the row
    4. The lower 8 bits of the row
 - **`0x03` = Draw monochrome image:** This command draws a rectangular image at the origin position, set beforehand by the "set origin position" command. The width and height of the image are 16 bit unsigned integer values, and communicated with the first four payload bytes:
    1. The upper 8 bits of the width
    2. The lower 8 bits of the width
    3. The upper 8 bits of the height
    4. The lower 8 bits of the height

    Then follows the image data, row for row left to right. This again is organized in packets as well, because we do length encoding of repeating pixel values. The beginning of such a packet, and therefore the first byte after the width+height information looks like `errr rrcc`, where each character represents one bit. The functions of the bits are as follows:
   - `cc`: The color to draw. These two bits are encoded as follows:
     - `00`: draw black
     - `01`: draw white
     - `10`: draw transparent (=do not change what's on the display)
     - `11`: invert what's on the display 
   - `rrrrr`: How many times we want to repeat this value, so if this is 0, we only draw one pixel in the respective color. Cuts down on the data to transmit considerably when there are larger areas of solid color. With all r-bits set to 1, we have 31 repeats, i.e. we can draw 32 consecutive pixels in the same color like this. It may be noted that these repetitions are designed to also wrap around into the next row when overflowing the image width.
   - `e`: extend repetitions. 32 consecutive pixels of the same color is not exactly much when the screen is not plastered with information. Therefore we want to enable more repetitions. The `e` bit indicates that the next byte holds seven more bits of repetition information, like this: `errr rrrr`. We then shift the repetition-bits we had so far to the left by 7, and make the 7 freed lsb bits our new 7 repetition bits. In that case, we would have 12 repetion bits alreay, allowing for 4096 consecutive pixels of the same color. We can extend up to three times, such that we get `5+7+7+7=26` repetition bits in total, giving us the ability to draw 67 million consecutive pixels of the same color, which is an order of magnitude more than what can fit on a 1600x1200 epaper display.
  
     So every time the msb bit is set, a repetition extender in the form of `errr rrrr` follows. Otherwise, a new color-information-byte in the form `errr rrcc` comes next. The controller decodes the length information and paints the pixels row for row left to right, while wrapping to the next row every time the image width is overflown. Lastly it is noteworthy that only images that fit on the screen are being drawn. This is because checking where image contents overflow the display introduces overhead while decoding, and that makes drawing images quite a bit slow. If one wants to show an image that doesn't fit on the screen on one or more sides, the image has to be cropped accordingly before sending it.
 - **`0x05` = Refresh display area:** The draw image command only changes the controler-internal framebuffer, but not anything on the screen yet. This has the advantage, that multiple changed areas can be sent before refreshing in one go. This command has column, row, width and height as parameters indicating the area to be refreshed. All of them are 16 bit unsigned integer values, and sent as follows as payload:
    1. The upper 8 bits of the column
    2. The lower 8 bits of the column
    3. The upper 8 bits of the row
    4. The lower 8 bits of the row
    5. The upper 8 bits of the width
    6. The lower 8 bits of the width
    7. The upper 8 bits of the height
    8. The lower 8 bits of the height
  
    It may be that the host computer wants to synchronize with the refresh command, such that it can start figuring out what image data to send next. The refresh display area command therefore sends the two bytes `0x05, 0x??` to the host, where the second questionmark-byte is the payload bytes 2, 4, 6 and 8 XORed. This answer is designed that way that the host can catch the correct answer message, as previous refreshes of different areas would yield a different questionmark-byte.
 - **`0x06` = Move cursor:** The controller firmware allows for drawing the mouse cursor on the screen itself. That way, only the cursor coordinates need to be communicated and no images of the cursor and previously covered areas. The command takes row and column of the cursor location, which are both 16 bit unsigned integer values. They get sent as payload like this:
    1. The upper 8 bits of the column
    2. The lower 8 bits of the column
    3. The upper 8 bits of the row
    4. The lower 8 bits of the row
  
    The cursor on the epaper display moves when the next "refresh display area" command is issued. It may be that no areas were issued new image data, in that case one can set all parameters to zero. The "refresh display area" command also takes care itself for updating areas where the cursor moved to / came from, as well as remembering pixels that get covered by the cursor.
