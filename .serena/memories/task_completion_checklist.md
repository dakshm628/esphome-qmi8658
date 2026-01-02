# Task Completion Checklist

## Before Completing a Task

### Code Quality
- [ ] Code follows the established naming conventions (see style_and_conventions.md)
- [ ] C++ code compiles without warnings
- [ ] Python code follows ESPHome component patterns
- [ ] Register addresses and constants are properly documented

### Testing
- [ ] Configuration compiles with `esphome compile example.yaml`
- [ ] For significant changes, test on actual hardware if possible

### Documentation
- [ ] Update README.md if adding new features or configuration options
- [ ] Add inline comments for complex logic
- [ ] Update example.yaml if configuration schema changes

### Version Control
- [ ] Stage relevant changes with `git add`
- [ ] Write descriptive commit message
- [ ] Use `--no-gpg-sign` flag when committing (per user preference)

## Notes

- This project has no automated linting or formatting tools configured
- No unit test framework (testing is done via ESPHome compilation and hardware testing)
- Since this is an ESPHome external component, the primary validation is successful compilation
