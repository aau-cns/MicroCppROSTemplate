# Micro C++ ROS 1 template

---

This script will allow to create from scratch a ROS1 workspace to be used with roscpp.
The scripts automatically create the folder structure, init the workspace, write the source files and build it
Both oop based development or simple (single) node type are supported.

---

## Clone

```bash
git clone https://github.com/AlessandroFornasier/MicroCppROSTemplate.git
```

## Setup

### Set permission
```bash
cd MicroCppROSTemplace
chmod +x create_package.py
```

### Set Personal info
Modify the file `resources/info.yaml` with your personal information

### Set file header
Modify the file `resources/HEADER` with the template of the header that should be included at the beginning of source files. Only the following keywords can be used `${YEAR}`, `${AUTHOR}` and `${EMAIL}`

## Set license
Modify the file `resources/LICENSE` with the LICENSE file that should be included with the package.

## Usage

```bash
./create_package.py -t <type> -w <path_to_workspace> -p <package_name> -n <node_name> -st </topic1> -sm <msg1> -pt </topic2> - pm <msg2>
```
