## Prepare dora for rk3568
Quickest way:
```
wget  https://github.com/dora-rs/dora/releases/download/v0.3.5/dora-v0.3.5-armv7-unknown-linux-musleabihf.zip
unzip dora-v0.3.5-armv7-unknown-linux-musleabihf.zip
```
Manual:
```bash
git clone https://github.com/dora-rs/dora.git
cd dora
docker pull messense/rust-musl-cross:armv7-musleabi
docker run --rm -it -v "$(pwd)":/home/rust/src messense/rust-musl-cross:armv7-musleabi bash
# in docker container
rustup target list
rustup target add armv7-unknown-linux-musleabi
cargo build -p dora-cli --release --target armv7-unknown-linux-musleabi
```
## Send dora to rk3568:
```bash
# in host machine
hdc file send target/armv7-unknown-linux-musleabi/release/dora /data
hdc shell
cd data
chmod +x dora
# start dora
./dora up 
./dora check
```
## Compile node and sink
```bash
cd dora-benchmark/dora-rs/rs-latency
cargo build --release --all
docker pull messense/rust-musl-cross:armv7-musleabi
docker run --rm -it -v "$(pwd)":/home/rust/src messense/rust-musl-cross:armv7-musleabi bash
# in docker container
cargo build --release --all --target armv7-unknown-linux-musleabi
# in host machine
hdc file send target/armv7-unknown-linux-musleabi/release/benchmark-example-sink /data
hdc shell

# in rk3568
cd /data
chmod +x benchmark-example-sink 
```
## Run remote dataflow

```bash
# in host machine
dora coordinator &
dora daemon --machine-id B &

# in rk3568
./dora daemon --machine-id A --coordinator-addr <host-ip>:53290 &

# in host machine
cd dora-benchmark/dora-rs/rs-latency
dora start remote_dataflow.yml

# in rk3568
cd /data
cat timer.csv
```
