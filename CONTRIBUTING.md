# Contribution Guidelines

## Introduction

INSTINCT started as a PhD project of mine but was developed to serve as a foundation for future research at our institute and also to be used and expanded by others. We are grateful to any help we can get!

/ Thomas (Institute of Navigation, University of Stuttgart, Germany)


## How to contribute to INSTINCT

If you want to help out you can:
- Answer questions on [Discussions](https://github.com/UniStuttgart-INS/INSTINCT/discussions)
- Contribute code
- Review & discuss [existing issues](https://github.com/UniStuttgart-INS/INSTINCT/issues)
- Review [Pull Requests](https://github.com/UniStuttgart-INS/INSTINCT/pulls)


## Discussion

You can ask questions, share screenshots and more at [GitHub Discussions](https://github.com/UniStuttgart-INS/INSTINCT/discussions).

If you want to make more discrete contact with the developers, please head over to our [website](https://instinct-software.de/) where you can find contact information.


## Filing an issue

[Issues](https://github.com/UniStuttgart-INS/INSTINCT/issues) are for bug reports and feature requests. Issues are not for asking questions (use [Discussions](https://github.com/UniStuttgart-INS/INSTINCT/discussions) for that).

Always make sure there is not already a similar issue to the one you are creating.

If you are filing a bug, please provide a way to reproduce it.

Make sure to use the GitHub Issue templates provided.


## Making a PR

For small things, just go ahead an open a PR. For bigger things, please file an issue first (or find an existing one) and announce that you plan to work on something. That way we will avoid having several people doing double work, and you might get useful feedback on the issue before you start working.

The documentation has a dedicated section for [Getting Started for developers](https://unistuttgart-ins.github.io/INSTINCT/Dev_Getting_Started.html).
If you contribute code, please try to write unit or integration tests as we try to improve on code coverage.

When you have something that works, open a draft PR. You may get some helpful feedback early!
Also, the [GitHub Actions](https://github.com/UniStuttgart-INS/INSTINCT/actions) will help you find errors.
When you feel the PR is ready to go, do a self-review of the code, and then open it for review.

Don't worry about having many small commits in the PR - they will be squashed to one commit once merged.

Please keep pull requests small and focused. The smaller it is, the more likely it is to get merged.


## PR review

Most PR reviews are done by us, but we very much appreciate any help we can get doing preliminary reviews of PRs!

When reviewing, we look for:
* The PR title and description should be helpful
* Breaking changes are documented in the PR description
* The code should be readable
* The code should have helpful documentation
* The code should follow the [Code Style](CONTRIBUTING.md#code-style)

## Code Style
We try to keep the repository in a uniform coding style. Therefore, please do the following

* Read some code before writing your own
* Use `clang-format` to format your code
* Leave the code cleaner than how you found it
* Use good names for everything
* Before making a function longer, consider adding a helper function
* Avoid double negatives
* Break the above rules when it makes sense

## Acknowledgement

These guidelines have been adapted from the [egui](https://github.com/emilk/egui) project. We are thankful to their author and if there are any problems with us reusing parts of it, please feel free to contact us.
- https://github.com/emilk/egui/blob/main/CONTRIBUTING.md
- https://github.com/emilk/egui/issues/3742