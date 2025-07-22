<a id="readme-top"></a>

<!-- LANGUAGE SWITCH -->

---

<!-- PROJECT SHIELDS -->

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

<p align="center">
    <a href="https://github.com/gdut-robocon/rc_controllers">View Demo</a>
    ·
    <a href="https://github.com/gdut-robocon/rc_controllers/issues/new?labels=bug&template=bug-report---.md">Report Bug</a>
    ·
    <a href="https://github.com/gdut-robocon/rc_controllers/issues/new?labels=enhancement&template=feature-request---.md">Request Feature</a>
</p>

<!-- TABLE OF CONTENTS -->

<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
        <li><a href="#configuration">Configuration</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>

---

## 📖 About The Project

This project provides a suite of ROS-based controllers for robotic systems, enabling precise motion control of chassis, shooter, gimbal, and action components. It supports omnidirectional and swerve drive kinematics, shooter wheel dynamics, gimbal pointing, and action command handling. Built on ROS Control and pluginlib, it offers modular, real-time safe control logic with YAML-based configuration, integrating Eigen for kinematic computations and leveraging TF2 for coordinate transformations.

### Key Features

- **ROS Control Integration**: Seamless integration with the ROS Control framework for modular and reusable controllers.
- **Modular Controller Plugins**: Supports plugin-based architecture for easy extensibility and runtime loading.
- **Omni-directional Motion Support**: Implements controllers for omnidirectional wheels with configurable geometry.
- **Swerve Drive Kinematics**: Advanced swerve drive control with independent module angle and speed control.
- **Real-time Command Handling**: Uses real-time safe buffers and control loops for deterministic behavior.
- **Torque Limited Effort Control**: Ensures safe operation by limiting torque based on physical constraints.
- **Dynamic Reconfiguration Support**: Allows runtime tuning of controller parameters.
- **TF2-based Gimbal Control**: Accurate coordinate transformation for 2-DOF gimbal control.
- **Chassis Velocity Transformation**: Transforms velocity commands from any coordinate frame to base frame.
- **Multi-mode Shooter Control**: Supports same-speed and differential-speed modes for robotic shooters.

### Built With

* [![ROS][ROS-shield]][ROS-url]
* [![C++][CPP]][CPP-url]
* [![Eigen][Eigen-shield]][Eigen-url]
* [![pluginlib][pluginlib-shield]][pluginlib-url]
* [![TF2][TF2-shield]][TF2-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### 📁 Project Structure

<details>
<summary>Click to expand project structure</summary>

```
rc_controllers/
├── .clang-format
├── .clang-tidy
├── action_controller/
│   ├── CMakeLists.txt
│   ├── action_controller_plugins.xml
│   ├── package.xml
│   ├── include/
│   │   └── action_controller/
│   │       ├── action_controller.h
│   ├── src/
│   │   └── action_controller.cpp
│   └── test/
│       ├── test_action_controller.launch
│       └── test_action_controller.yaml
├── chassis_controllers/
│   ├── CMakeLists.txt
│   ├── chassis_controllers_plugins.xml
│   ├── package.xml
│   ├── include/
│   │   └── chassis_controllers/
│   │       ├── chassis_base.h
│   │       ├── omni.h
│   │       └── swerve.h
│   ├── src/
│   │   ├── chassis_base.cpp
│   │   ├── omni.cpp
│   │   └── swerve.cpp
│   └── test/
│       ├── test_omni.launch
│       ├── test_omni.yaml
│       ├── test_swerve.launch
│       └── test_swerve.yaml
├── rc_gimbal_controller/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── rc_gimbal_controller_plugins.xml
│   ├── include/
│   │   └── rc_gimbal_controller/
│   │       ├── gimbal_base.h
│   ├── src/
│   │   └── gimbal_base.cpp
│   └── test/
│       ├── gimbal_config_template.yaml
│       └── load_controllers.launch
├── rc_shooter_controller/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── rc_shooter_controller_plugins.xml
│   ├── include/
│   │   └── rc_shooter_controller/
│   │       ├── rc_shooter_controller.h
│   ├── src/
│   │   └── rc_shooter_controller.cpp
│   └── test/
│       ├── load_controller.launch
│       └── rc_shooter_controller_template.yaml
```

</details>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## 🚀 Getting Started

This is an example of how you may give instructions on setting up your project locally. To get a local copy up and running follow these simple steps.

### Prerequisites

- ROS Noetic or newer installed
- catkin workspace initialized
- `pluginlib`, `controller_interface`, `tf2`, `Eigen3`, `realtime_tools` installed

### Installation

1. Clone the repository into your catkin workspace:
   
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/gdut-robocon/rc_controllers.git
   ```

2. Build the package:
   
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

3. Source the setup file:
   
   ```bash
   source devel/setup.bash
   ```

### Configuration

- Each controller supports YAML-based configuration for parameters such as wheel geometry, joint limits, publishing rate, and timeout.
- Example configuration files are provided in the `test/` directories of each controller package.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## 💻 Usage

- Launch the controller using the provided launch files in the `test/` directories.
- For example, to test the omni-directional controller:
  
  ```bash
  roslaunch rc_controllers test_omni.launch
  ```
- Controllers can be dynamically reconfigured using `dynamic_reconfigure`.
- Controllers can be loaded via `controller_manager` and configured using YAML files.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## 🗺️ Roadmap

- Add support for differential drive controllers
- Enhance shooter controller with PID-based speed control
- Implement advanced trajectory tracking for chassis controllers
- Add simulation support in Gazebo
- Improve documentation and add tutorials

See the [open issues](https://github.com/gdut-robocon/rc_controllers/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## 🤝 Contributing

Contributions are what make the open-source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Top contributors:

<a href="https://github.com/gdut-robocon/rc_controllers/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=gdut-robocon/rc_controllers" alt="contrib.rocks image" />
</a>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## 🎗 License

Copyright © 2024-2025 [rc_controllers][rc_controllers].  
Released under the [MIT][license-url] license.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## 📧 Contact

Email: jialonglongliu@gmail.com  
Project Link: [https://github.com/gdut-robocon/rc_controllers](https://github.com/gdut-robocon/rc_controllers)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## 🙌 Acknowledgments

- ROS community for providing the ROS Control framework
- Eigen developers for high-performance linear algebra library
- TF2 developers for coordinate transformation utilities
- Contributors and users of this project

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

<!-- MARKDOWN LINKS & IMAGES -->

[rc_controllers]: https://github.com/gdut-robocon/rc_controllers
[license-url]: https://github.com/gdut-robocon/rc_controllers/blob/master/LICENSE.txt

<!-- Tech Stack -->

[ROS-shield]: https://img.shields.io/badge/ROS-217F85?style=flat-round&logo=ros&logoColor=white
[ROS-url]: https://www.ros.org/
[CPP]: https://img.shields.io/badge/C++-00599C?style=flat-round&logo=cplusplus&logoColor=white
[CPP-url]: https://en.wikipedia.org/wiki/C%2B%2B
[pluginlib-shield]: https://img.shields.io/badge/pluginlib-6CB040?style=flat-round&logo=ros&logoColor=white
[pluginlib-url]: http://wiki.ros.org/pluginlib
[TF2-shield]: https://img.shields.io/badge/TF2-FF6600?style=flat-round&logo=ros&logoColor=white
[TF2-url]: http://wiki.ros.org/tf2
[Eigen-shield]: https://img.shields.io/badge/Eigen-922790?style=flat-round&logo=eigen&logoColor=white
[Eigen-url]: https://eigen.tuxfamily.org/

<!-- MARKDOWN LINKS & IMAGES -->

[contributors-shield]: https://img.shields.io/github/contributors/gdut-robocon/rc_controllers.svg?style=flat-round
[contributors-url]: https://github.com/gdut-robocon/rc_controllers/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/gdut-robocon/rc_controllers.svg?style=flat-round
[forks-url]: https://github.com/gdut-robocon/rc_controllers/network/members
[stars-shield]: https://img.shields.io/github/stars/gdut-robocon/rc_controllers.svg?style=flat-round
[stars-url]: https://github.com/gdut-robocon/rc_controllers/stargazers
[issues-shield]: https://img.shields.io/github/issues/gdut-robocon/rc_controllers.svg?style=flat-round
[issues-url]: https://github.com/gdut-robocon/rc_controllers/issues
[release-shield]: https://img.shields.io/github/v/release/gdut-robocon/rc_controllers?style=flat-round
[release-url]: https://github.com/gdut-robocon/rc_controllers/releases
[release-date-shield]: https://img.shields.io/github/release-date/gdut-robocon/rc_controllers?color=9cf&style=flat-round
[license-shield]: https://img.shields.io/github/license/gdut-robocon/rc_controllers.svg?style=flat-round
[license-url]: https://github.com/gdut-robocon/rc_controllers/blob/master/LICENSE.txt

<!-- Tech Stack -->

<!-- 编程语言 -->

[Python]: https://img.shields.io/badge/Python-3776AB?style=flat-round&logo=python&logoColor=white
[Python-url]: https://www.python.org/
[JavaScript]: https://img.shields.io/badge/JavaScript-F7DF1E?style=flat-round&logo=javascript&logoColor=black
[JavaScript-url]: https://developer.mozilla.org/en-US/docs/Web/JavaScript
[TypeScript]: https://img.shields.io/badge/TypeScript-007ACC?style=flat-round&logo=typescript&logoColor=white
[TypeScript-url]: https://www.typescriptlang.org/
[Java]: https://img.shields.io/badge/Java-ED8B00?style=flat-round&logo=openjdk&logoColor=white
[Java-url]: https://www.oracle.com/java/
[Go]: https://img.shields.io/badge/Go-00ADD8?style=flat-round&logo=go&logoColor=white
[Go-url]: https://golang.org/
[Rust]: https://img.shields.io/badge/Rust-000000?style=flat-round&logo=rust&logoColor=white
[Rust-url]: https://www.rust-lang.org/
[C]: https://img.shields.io/badge/C-00599C?style=flat-round&logo=c&logoColor=white
[C-url]: https://en.wikipedia.org/wiki/C_(programming_language)
[CPP]: https://img.shields.io/badge/C++-00599C?style=flat-round&logo=cplusplus&logoColor=white
[CPP-url]: https://en.wikipedia.org/wiki/C%2B%2B
[CSharp]: https://img.shields.io/badge/C%23-239120?style=flat-round&logo=csharp&logoColor=white
[CSharp-url]: https://docs.microsoft.com/en-us/dotnet/csharp/
[MATLAB]: https://img.shields.io/badge/MATLAB-0076A8?style=flat-round&logo=mathworks&logoColor=white
[MATLAB-url]: https://www.mathworks.com/products/matlab.html

<!-- 前端框架 -->

[React.js]: https://img.shields.io/badge/React-20232A?style=flat-round&logo=react&logoColor=61DAFB
[React-url]: https://reactjs.org/
[Vue.js]: https://img.shields.io/badge/Vue.js-35495E?style=flat-round&logo=vuedotjs&logoColor=4FC08D
[Vue-url]: https://vuejs.org/
[Angular.io]: https://img.shields.io/badge/Angular-DD0031?style=flat-round&logo=angular&logoColor=white
[Angular-url]: https://angular.io/
[Next.js]: https://img.shields.io/badge/next.js-000000?style=flat-round&logo=nextdotjs&logoColor=white
[Next-url]: https://nextjs.org/

<!-- 后端框架 -->

[Flask]: https://img.shields.io/badge/Flask-000000?style=flat-round&logo=flask&logoColor=white
[Flask-url]: https://flask.palletsprojects.com/
[Django]: https://img.shields.io/badge/Django-092E20?style=flat-round&logo=django&logoColor=white
[Django-url]: https://www.djangoproject.com/
[FastAPI]: https://img.shields.io/badge/FastAPI-005571?style=flat-round&logo=fastapi&logoColor=white
[FastAPI-url]: https://fastapi.tiangolo.com/
[Express.js]: https://img.shields.io/badge/Express.js-404D59?style=flat-round&logo=express&logoColor=white
[Express-url]: https://expressjs.com/
[Spring]: https://img.shields.io/badge/Spring-6DB33F?style=flat-round&logo=spring&logoColor=white
[Spring-url]: https://spring.io/
[Node.js]: https://img.shields.io/badge/Node.js-43853D?style=flat-round&logo=node.js&logoColor=white
[Node-url]: https://nodejs.org/

<!-- AI/ML 相关 -->

[OpenAI]: https://img.shields.io/badge/OpenAI-000000?style=flat-round&logo=openai&logoColor=white
[OpenAI-url]: https://openai.com/
[Rich]: https://img.shields.io/badge/Rich-000000?style=flat-round&logo=rich&logoColor=white
[Rich-url]: https://rich.readthedocs.io/

<!-- 数据库 -->

[PostgreSQL]: https://img.shields.io/badge/PostgreSQL-316192?style=flat-round&logo=postgresql&logoColor=white
[PostgreSQL-url]: https://www.postgresql.org/
[MySQL]: https://img.shields.io/badge/MySQL-00000F?style=flat-round&logo=mysql&logoColor=white
[MySQL-url]: https://www.mysql.com/
[MongoDB]: https://img.shields.io/badge/MongoDB-4EA94B?style=flat-round&logo=mongodb&logoColor=white
[MongoDB-url]: https://www.mongodb.com/
[Redis]: https://img.shields.io/badge/Redis-DC382D?style=flat-round&logo=redis&logoColor=white
[Redis-url]: https://redis.io/
[SQLite]: https://img.shields.io/badge/SQLite-07405E?style=flat-round&logo=sqlite&logoColor=white
[SQLite-url]: https://www.sqlite.org/

## REFERENCE 
[rm_controller](https://github.com/rm-controls/rm_controllers)