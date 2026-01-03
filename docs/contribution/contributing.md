# Contributing to KLASK Hardware

Thank you for your interest in contributing to the KLASK Hardware! We welcome contributions from the community to help improve and expand the project.

## Development Workflow

We use a fork-and-pull-request workflow with short-lived feature branches.

### 1. Fork and Clone

Fork the repository on GitHub and clone your fork locally. Add the original repository as an upstream remote.

### 2. Create a Feature Branch

Create a descriptive branch for your work. Common prefixes:

- `feature/` – New features
- `fix/` – Bug fixes
- `docs/` – Documentation
- `refactor/` – Code refactoring
- `test/` – Tests
- `chore/` – Maintenance

### 3. Make Your Changes

- Write clean, well-documented code
- Follow the [coding guidelines](code_guideline.md)
- Test your changes thoroughly
- Keep commits focused and atomic

### 4. Commit with Conventional Commits

We use [Conventional Commits](https://www.conventionalcommits.org/) for clear history and automated changelogs.

**Format:** `<type>(<scope>): <description>`

**Types:** `feat`, `fix`, `docs`, `style`, `refactor`, `perf`, `test`, `build`, `ci`, `chore`

**Examples:**

- `feat(controller): add MPC balancing controller`
- `fix(estimator): correct quaternion normalization`
- `docs(tutorials): add system-id workflow guide`

### 5. Keep Updated and Push

Regularly sync your branch with the upstream main branch, then push to your fork.

### 6. Open a Pull Request

Before submitting:

- [ ] Code follows the [coding guidelines](code_guideline.md)
- [ ] Commits follow [Conventional Commits](https://www.conventionalcommits.org/)
- [ ] All tests pass (if applicable)
- [ ] Documentation is updated (if needed)
- [ ] Branch is up to date with `main`

Provide a clear description:

- **What** – What does this PR do?
- **Why** – Why is this change needed?
- **How** – How does it work? (for complex changes)
- **Testing** – How was this tested?
- **Related Issues** – Link any related issues (e.g., "Closes #123")

A maintainer will review your PR. You can push updates to address feedback. Once approved, your PR will be merged.

---

## Development Setup

Follow the [Environment Setup Tutorial](../tutorials/03-dev-setup.md) to get started with your local environment and have a look at the other [tutorials](../tutorials/00-overview.md) for more information on working with KLASK Hardware.

## Questions and Support

- **Documentation**: Check the [docs](../index.md) folder first
- **Issues**: Search existing issues for similar questions

## License

By contributing, you agree that your contributions will be licensed under the same license as the project.

---

Thank you for contributing to KLASK Hardware!
