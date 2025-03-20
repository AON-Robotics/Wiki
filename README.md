# AON Robotics Wiki

Welcome to the AON Robotics Wiki repository! This repository is dedicated to building and organizing a modular codebase for the various components of our robots. Each component of the robot, such as the drivetrain, claw, intake, conveyor, flaps, arms, and others, will have its own folder and README file to explain how it works and how to use it.

## Repository Structure

The codebase is organized as follows:

```
/include/aon/
├── drivetrain/
│   ├── src/
│   ├── README.md
├── claw/
│   ├── src/
│   ├── README.md
├── intake/
│   ├── src/
│   ├── README.md
├── component/
│   ├── src/
│   ├── README.md
...
```

Each folder within `/include/aon/` corresponds to a robot component and contains the following:
- The code for the component.
- A `README.md` that explains:
  - **How the component works**: An overview of the functionality.
  - **How to use it**: Instructions on how to integrate or control the component within the robot.

## Guidelines for Contributing

1. **Creating a Component**: If you're adding a new robot component, create a new folder inside `/include/aon/` with the component's name.
2. **Documenting the Component**: Ensure that the folder includes a `README.md` with clear explanations of how the component works and any specific usage instructions.
3. **Code Organization**: Keep the code for each component modular and well-commented to help others understand its purpose and functionality.
4. **Consistency**: Follow the same format for each component's `README.md` to maintain consistency across the project.

## Purpose

This wiki repository serves as a central place for documenting the modular components that make up the robot codebase. It helps ensure that each part of the robot is well-documented and easily reusable for current and future projects.

By maintaining modularized components with clear documentation, we can promote collaboration, improve understanding of each robot part, and streamline the development process.

If you have any questions or need help, feel free to reach out to the team!

---

AON Robotics | Building the Future of Robotics
