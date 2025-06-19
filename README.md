# Pico Neuron Serial

Repository that implements a C/C++ program that reads a serial tty port in realtime.

## Description

This repository provides code to read with a realtime thread a USB port.

## Getting Started

### Prerequisites

- USB Port
- C/C++
- Preempt-RT kernel

### Building

```sh
git clone https://github.com/sergiohidalgo818/pico-neuron-serial.git
cd pico-neuron-serial
cmake -B build .
cd build
make -j4
cd ..
sudo bash scripts/give_rt_privs.bash
```

### Usage

The program accepts several command line arguments to configure its behavior. Below is a description of each supported argument:

| Argument                | Type   | Default Value        | Description                                                                                   |
|-------------------------|--------|----------------------|-----------------------------------------------------------------------------------------------|
| `-d`, `--directory`     | string | `"data/"`            | Specifies the directory where the output file will be located.                                       |
| `-f`, `--filename`      | string | `"hindmarsh-rose.csv"` | Name of the output file.                                                             |
| `-s`, `--separator`     | string | `" "`                | Field separator used when writting the file (e.g., space, comma, tab).                         |
| `-sn`, `--serial-name`  | string | `"/dev/ttyUSB0"`     | Name or path of the serial port device to read data.                              |
| `-sr`, `--serial-rate`  | int    | `1000000`            | Baud rate (speed) for the serial communication.                                               |
| `-rm`, `--real-measure` | flag   | `false`              | If provided, enables "real measure" mode (no parameter needed; just include the flag in call).|


```bash
./build/
```


## Scripts

### give_rt_privs.bash

Gives realtime privilege to program binary.

### plot_model.py

Plots the data, use with `-h` for viewing the arguments.
