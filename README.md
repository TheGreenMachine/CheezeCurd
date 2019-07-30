# CheezeCurd
A learning repo based off of The Cheesy Poofs codebase

- The main libs were left pretty much intact and most of the main robot code remained the same, just changed the package names and talon ids.

- Part of this refactor included enhancing the TalonSRXFactory.  I created a concept of a GhostTalonSRX.java this is a dummy TalonSRX that has the same interface as a TalonSRX but does pretty much nothing.  This allows the code to run as as without having the exact same robot.  You just put a -1 for any can devices that you do not have.  See Constants.java

- The original repo was converted to use the 2019 control system
