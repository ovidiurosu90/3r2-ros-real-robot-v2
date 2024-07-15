[//]: # (Some of the lines end with 2 spaces. This is the way to specify a line break in markdown)

# Wheel Reduction

ODrive updated their firmware and starting with v0.5.1 **'encounter counts' changed to 'turns'**. For more info check ODrive's [Migration Guide](https://docs.odriverobotics.com/migration).

In the original release from James Bruton (commit on Nov 3, 2020), the encoder units were encoder counts. Now we use turns.

## Hardware

We use ODrive's [CUI AMT102](https://eu.odriverobotics.com/shop/cui-amt-102) capacitive encoders, which have 8192 counts per resolution.

The motor pulleys have 15 teeth. The wheel pulleys have 96 teeth.  
96/15 = 6.4 => 15:96 reduction == **6.4:1** reduction

The wheels have 200mm in diameter. Circumference C = 2 * PI * radius.  
C = 2 * 3.141592 * 100 => **628.318530** mm calculated circumference (approximated by James at 628.32; 633 in reality).

The distance between wheels is **346** mm in the 3d model (353 mm in reality).


## Driving Forward

1 revolution of the wheel means 628.318530 mm (approximated by James at 628.32) travelled.

The motor needs to turn 6.4 times to make a full revolution (because of the 6.4:1 reduction).

1 wheel revolution = 6.4 motor turns = 6.4 * 8192 encoder counts = 52428.8 encoder counts (approximated by James at 52428).

1 wheel revolution = 628.318530 mm => for each travelled mm we have 52428.8 / 628.318530 encoder counts.  
52428.8 / 628.318530 = 83.443026 encoder counts per mm (approximated by James to 83.44, value in his code)

6.4 motor turns / 628.318530 mm = **0.010185** turns per mm (value in my code based on 3d model measurements)


### Calculate the number of encoder counts / turns we need in order to travel at 1 meter per second

1 revolution ... 628.318530 mm  
x revolution ... 1000 mm (1m)  
x = 1 * 1000 / 628.318530 = 1.591549 revolutions per m (approximated by James to 1.592)

1.591549 revolutions per m * 52428.8 encoder counts = 83443.004211 encoder counts per m  
1.591549 revolutions per m * 6.4 turns = 10.185913 turns per m

each meter / sec = 83443.004211 encoder counts (approximated by James to 83466, value in his code)  
each meter / sec = **10.185916** turns (value in my code based on 3d model measurements)


## Turning

360 degrees = 2 * PI radians  
180 degress = PI radians


### Distance each wheel has to travel in order to turn 180 degrees

346 mm (distance between wheels) * PI / 2 (number of wheels)  
= 346 * 3.141592 / 2 = 543.495416 mm (approximated by James to 565.48 mm)

1 revolution ... 628.318530 mm  
x revolution ... 543.495416 mm  
x = 1 * 543.495416 / 628.318530 = 0.864999 revolutions

0.864999 revolutions * 6.4 motor turns = 5.535998 motor turns (NOTE James has 5.78 here)  
5.535998 * 8192 = 45350.902616 encoder counts (approximated by James to 47411)

1 radian = 45350.902616 / PI = 14435.643653 encoder counts (approximated by James to 15091, value in his code)  
1 radian = 5.535998 / PI = **1.762163** turns (value in my code based on 3d model measurements)

