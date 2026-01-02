# Suggested Commands

## ESPHome Development

### Compile and validate configuration
```bash
esphome compile example.yaml
```

### Upload to device
```bash
esphome upload example.yaml
```

### View logs
```bash
esphome logs example.yaml
```

### Run ESPHome dashboard
```bash
esphome dashboard .
```

## Git Commands

### Commit changes (no GPG signing as per user preference)
```bash
git add .
git commit --no-gpg-sign -m "Your commit message"
```

### View commit history
```bash
git log --oneline
```

## System Utilities (Linux)

### List files
```bash
ls -la
```

### Find files
```bash
find . -name "*.py"
find . -name "*.cpp" -o -name "*.h"
```

### Search in files
```bash
grep -r "pattern" components/
```

## Notes

- This project uses ESPHome's external component system
- The component requires I2C to be configured in the ESPHome YAML
- Default I2C address is 0x6B
- Testing is done by compiling and uploading to actual hardware
- No unit test framework is configured (typical for ESPHome components)
