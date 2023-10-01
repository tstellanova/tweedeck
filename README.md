# tweedeck

Experiments exercisting the [Lilygo T-Deck](https://www.lilygo.cc/products/t-deck) using embedded, `no_std` rust.
This utilizes the (WIP) espressif tools for no-std rust.


## Running Examples

When developing with T-Deck, I have `cargo espflash` installed and use:

```shell
cargo build --example headless && cargo espflash flash --example headless
```


Lorawan is in progress and currently does not successfully receive or send.
It also relies on a heavily modified sx126x crate, which has not been pushed to git. 

```shell
cargo build --features "lorawan" --example lora_recv_log && cargo espflash flash --features "lorawan" --example lora_recv_log --monitor
```


