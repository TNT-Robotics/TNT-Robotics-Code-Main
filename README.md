
# TNT Robotics Code Documentation
#### Written by [Tomas Pokorny](  mailto:tomasekpokorny02@gmail.com?subject=TNT%20Robotics%20Code)
##### Last updated - 19/10/2022

## [! Original ReadMe with FTC Notes !](https://github.com/TNT-Robotics/TNT-Robotics-Code-Main/blob/main/FTCReadMe.md)

## Contents
- [Basic driving functions](#example)
- [Advanced driving functions](#example2)
- [Arm functions](#third-example)
- [Vision functions](#fourth-examplehttpwwwfourthexamplecom)

` 
| Item | Value | Qty | No |
| :------- | ----: | :---: |
| Computer | $1600 | 5 |

```java
    public void strafeLeft(double power, int time) {
        newFunctions.updateTele("Strafing left with power " + power + " for " + time + "ms.", 0);
        cfg.getRfD().setPower(power);
        cfg.getLfD().setPower(power * -1);
        cfg.getLbD().setPower(power);
        cfg.getRbD().setPower(power * - 1);
        newFunctions.slp(time);
        dontMove();
    }

````
