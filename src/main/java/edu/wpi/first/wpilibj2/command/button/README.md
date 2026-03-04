This file is changed to allow us to modify the WPILib Trigger Class

* We add overloaded and() and or() methods that can take any number of arguments
* We changed the initial state of the triggers to be true or false based on the binding, so they can immediately run if the condition is met. This is important for having triggers for each robot state (disabled, teleop, etc.)
* To deploy this to a robot you have to edit the build.gradle file -> The Jar duplicateStrategy should be set to "duplicatesStrategy = DuplicatesStrategy.EXCLUDE"
