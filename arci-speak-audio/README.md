# arci-speak-audio

[![crates.io](https://img.shields.io/crates/v/arci-speak-audio.svg)](https://crates.io/crates/arci-speak-audio) [![docs](https://docs.rs/arci-speak-audio/badge.svg)](https://docs.rs/arci-speak-audio)

[`arci::Speaker`](https://docs.rs/arci/*/arci/trait.Speaker.html) implementation for playing audio files.

## Dependencies

### Linux

This package use [rodio](https://github.com/RustAudio/rodio) which uses ALSA to play sound in Linux.
So `libasound2-dev` package (debian or ubuntu) is required

```bash
sudo apt install libasound2-dev
```
