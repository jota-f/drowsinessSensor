# Contribution Guide for drowsinessSensor

Thank you for considering contributing to drowsinessSensor! This document provides guidelines for contributing to the project.

## How to Contribute

### Reporting Bugs

If you find a bug, please create an issue on GitHub following these steps:

1. Check if the bug has already been reported
2. Use the bug report template and provide:
   - Clear description of the problem
   - Steps to reproduce
   - Expected behavior
   - Observed behavior
   - Information about your environment (Arduino IDE version, ESP32-CAM, etc.)
   - Screenshots or logs, if possible

### Suggesting Improvements

To suggest improvements:

1. Check if your idea has already been suggested
2. Create an issue detailing:
   - Clear description of the improvement
   - Why this improvement would be useful
   - How to implement it (if you have technical suggestions)

### Submitting Pull Requests

1. Fork the repository
2. Create a branch for your feature (`git checkout -b feature/feature-name`)
3. Make your changes
4. Verify that the code compiles and test on hardware, if possible
5. Commit your changes (`git commit -m 'Add new feature'`)
6. Push to the branch (`git push origin feature/feature-name`)
7. Open a Pull Request

## Code Guidelines

### Code Style

- Indent your code with 2 spaces
- Use descriptive variable and function names
- Add comments for complex code
- Keep functions small and with a single purpose
- Follow the naming convention already present in the code

### Documentation

- Document all new functions and parameters
- Update documentation when modifying existing functionality
- Add comments explaining the "why" when the code is not self-explanatory

### Testing

- Test your changes on real hardware (ESP32-CAM)
- Check different lighting conditions, if relevant
- Ensure that your changes do not exceed the available memory on the ESP32-CAM

## Project Resources

- Memory: The ESP32-CAM has limited resources; be economical with RAM
- CPU: Optimize for the best possible performance
- PSRAM: Use when available for large buffers

## Optimizations

When contributing, consider:

- Minimizing memory usage
- Reducing processing time
- Maintaining detection quality
- Prioritizing system stability

## Review Process

1. A maintainer will review your Pull Request
2. There may be requests for changes or clarifications
3. Once approved, your PR will be merged into the project

## Acknowledgements

We appreciate your contribution to making drowsinessSensor better! 