
# TNT Robotics Code Documentation
#### Written by [Tomas Pokorny](  mailto:tomasekpokorny02@gmail.com?subject=TNT%20Robotics%20Code)
##### Last updated - 19/10/2022

## [! Original ReadMe with FTC Notes !](https://github.com/TNT-Robotics/TNT-Robotics-Code-Main/blob/main/FTCReadMe.md)

## Table of Contents
- [Basic driving functions](#Basic%20driving%20functions)
- [Advanced driving functions](#example2)
- [Arm functions](#third-example)
- [Vision functions](#fourth-examplehttpwwwfourthexamplecom)

## Basic driving functions


#### GoForward (double power, int time)
This
| Action   | Type    |
| ---------| ------- |
| Input    | Double  |
| Output   | Void    |





<details>
<summary>

*List of groceries*
</summary>

* Vegetables
</details>

java
	    public void strafeLeft(double power, int time) {
	        newFunctions.updateTele("Strafing left with power " + power + " for " + time + "ms.", 0);
	        cfg.getRfD().setPower(power);
	        cfg.getLfD().setPower(power * -1);
	        cfg.getLbD().setPower(power);
	        cfg.getRbD().setPower(power * - 1);
	        newFunctions.slp(time);
	        dontMove();
	    }
