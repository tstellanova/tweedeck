# tweedeck

Experiments exercisting the [Lilygo T-Deck](https://www.lilygo.cc/products/t-deck) using embedded, `no_std` rust.
This utilizes the (WIP) espressif tools for no-std rust.


## Running Examples

When developing with T-Deck, I have `cargo espflash` installed and use:

```shell script
cargo build --example headless && cargo espflash flash --example headless
```


