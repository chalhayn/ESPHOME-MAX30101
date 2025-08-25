
# ESPHome External Component: MAX30101 (flat layout, local include)

Layout is **components/ at repo root** so ESPHome can fetch it cleanly via `external_components`.

## Repo structure
```
components/
  max30101/
    __init__.py
    sensor.py
    max30101_component.h
    max30101_component.cpp
    MAX30101_PulseOximeter.h
    MAX30101_PulseOximeter.cpp
```

## ESPHome YAML
```yaml
external_components:
  - source:
      type: git
      url: https://github.com/Chalhayn/ESPHOME-MAX30101
      ref: main
    components: [max30101]
    refresh: 0s   # force refetch once; remove later
```
