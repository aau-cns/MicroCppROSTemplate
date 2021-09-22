# Micro C++ ROS 1 template

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
