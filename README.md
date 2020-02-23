# usb-paddle
Adaptor to make your Iambic Paddle a USB keyboard input


# Original Idea
Full credit for the design on this goes to these guys:  https://www.nycresistor.com/2012/02/20/morse-code-keyboard/

But everytime I get asked about it, I have to go hunt this page down, so I'm consilidating my notes here.  

You can see my comment on the bottom of the page where I detail the very minor touch ups I've done here.

I hope this helps somebody in the future that wants to build this little guy.  Its a fantastic toy and really makes practice easy.

# The build

I ordered a Teensy 2.0 off of amazon fairly easy enough.  The code loaded on much like any other Arduino software would load.  Its a quite elegant design that uses some pins for input, some pins for speaker output and re-uses the USB port to connect to the computer. I took a few extra steps and put a 1/8in stereo jack on it and dropped the works into a case for the earbuds that came with a samsung phone of mine.

Edit `iambic.c` to set key speed

`make` and then open the Arduino IDE and setup for your Teensy and download the `.hex` file.

After that, plug into your USB port, plug in your Paddle and get to keying!

# Wiring

Paddle gets connected to B4, B5 and GND pins. B4 is dit, B5 is dah - swap if that's your thing.

Speaker gets plugged into VCC and pin OC0A which was labled B7 on the board. It was labeled as OC0A in the Teensy docs.

# The video

Video link here: https://youtu.be/SAqfvn5xvzI


