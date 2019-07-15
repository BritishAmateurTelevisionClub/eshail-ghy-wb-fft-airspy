# airspy_fft_ws

A daemon to serve an FFT over websocket

## Download

```
git clone --recursive https://github.com/BritishAmateurTelevisionClub/eshail-ghy-wb-fft-airspy.git
```

## Install libairspy dependency

```
sudo apt install libairspy-dev
```


## Build libwebsockets dependency (no system install)

```
cd libwebsockets/
mkdir build/
cd build/
cmake .. -DLWS_WITH_SSL=OFF
make
```

## Compile

```
make
```

## Install as systemd service

```
./install
```

# Copyright 2019 British Amateur Television Club
