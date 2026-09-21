# Build and run

[中文](README_ZH.md)

Requires Linux, a C++17 compiler, Make, and Boost.Program_options development
files (including the static library). Run commands from this directory.

On Ubuntu/Debian, install the dependencies and compile:

```bash
sudo apt-get install build-essential libboost-program-options-dev
make
```

Run the included example:

```bash
./focbs -i tools/example.mapfua -t 60
```

To regenerate the example, use Python 3 (standard library only):

```bash
python3 tools/gen_mapfua.py --map tools/random-32-32-20.map \
  --num-assigned 4 --num-unassigned 20 --seed 0 --output tools/example.mapfua
```

Adjust the counts and seed as needed. To use another input, replace
`tools/example.mapfua` with its path.

A relative map path inside the input file is resolved from that file's directory.
Alternatively, use a map and scenario file; replace `10` with the required count:

```bash
./focbs --map /path/to/map.map --agents /path/to/tasks.scen --agentNum 10 -t 60
```

`-t` sets the time limit in seconds. To see all command-line options:

```bash
./focbs --help
```
