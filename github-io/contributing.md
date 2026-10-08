---
layout: doc
title: Contributing
permalink: /github-io/contributing/
lede: Vision Pilot is developed in the open by the Privately Owned Vehicle working group. Here is how to join in.
description: How to onboard, join the working group, report bugs and submit pull requests to Vision Pilot.
---

Vision Pilot is built by the **Privately Owned Vehicle working group** of the Autoware Foundation.
Everything - design discussion, meetings, minutes, code review - happens in public.

## Onboarding, in four steps

1. **Star the [repository]({{ site.links.repo }})** and read this documentation to get a feel for
   the project's goals.
2. **Join the [Autoware Discord]({{ site.links.discord }})** and introduce yourself in the
   [`privately owned vehicles` channel]({{ site.links.discord_channel }}). A short message about
   your background and what you would like to work on is enough.
3. **Catch up on recent discussions.** Meeting minutes and recordings are posted to
   [GitHub Discussions]({{ site.links.discussions }}).
4. **Join a working group meeting.** Weekly, on Mondays, in two slots.

### Working group meetings

Both slots cover the same agenda, so attend whichever suits your timezone:

| Slot | Best for | Calendar |
| --- | --- | --- |
| **Slot 1** | East Asia | [Add to calendar](https://calendar.google.com/calendar/u/0/r/week/2024/11/18?eid=MzlmZDZvNjhjZ3FwOXZkMjc4cHZqbHBhaDhfMjAyNDExMThUMDQzMDAwWiBhdXRvd2FyZS5vcmdfNmxvbDBobzVmdDAyMTdoOGM2MHBpMWZtMzBAZw) |
| **Slot 2** | Europe / Americas | [Add to calendar](https://calendar.google.com/calendar/u/0/r/week/2024/11/18?eid=aG0yMWUzNXU1N2JxYW9wMHZkb2lncmg5bGNfMjAyNDExMThUMTYwMDAwWiBhdXRvd2FyZS5vcmdfNmxvbDBobzVmdDAyMTdoOGM2MHBpMWZtMzBAZw) |

## Ways to contribute

**Discussions.** Propose features, weigh in on open threads, organise discussions, and answer
other contributors' questions in
[Autoware Discussions](https://github.com/orgs/autowarefoundation/discussions).

**Bug reports.** Search the [issue tracker]({{ site.links.issues }}) first - someone may have hit
it already and found a workaround. When filing, include a **minimal set of steps to reproduce**.
If you plan to fix it yourself, discuss the approach in the issue before opening a pull request.

**Pull requests.** Small changes can go straight to a PR:

- documentation updates
- spelling fixes
- CI failures
- compiler or static-analysis warnings
- small changes within a single package

For anything larger, follow the four-step process:

1. [Create a GitHub Discussion](https://github.com/orgs/autowarefoundation/discussions) proposing
   the change, so maintainers can confirm it fits the design direction.
2. [Create an issue]({{ site.links.issues }}) once there is consensus.
3. Open a pull request referencing that issue.
4. Add documentation for the change - including on this site, if it affects users.

## Pull request checks

Two CI workflows gate every pull request:

### `semantic-pull-request`

Your **PR title** must follow
[Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/):

```
feat(visualization): add occupancy grid colour configuration
fix(engine): guard against null ONNX session on CPU provider
docs: document rrd logging keys
```

### `pre-commit`

[pre-commit](https://pre-commit.ci/) runs the project's formatters and linters. Install the hooks
locally so you catch issues before pushing:

```bash
pip install pre-commit
pre-commit install
pre-commit run --all-files
```

The repository configures `clang-format` (see `.clang-format`), `cpplint` (`CPPLINT.cfg`),
`markdownlint`, `prettier`, `yamllint` and `cspell`. If `cspell` objects to a legitimate term, add
it to `.cspell.json` in the same PR.

See the Autoware
[pull request guidelines](https://autowarefoundation.github.io/autoware-documentation/main/contributing/pull-request-guidelines/)
and [license notations](https://autowarefoundation.github.io/autoware-documentation/main/contributing/license/)
for the full house style.

## Contributing to this site

The documentation site lives in [`github-io/`]({{ site.links.repo }}/tree/main/github-io) and is a Jekyll
site published by GitHub Pages. Every page has an **Edit this page** link at the bottom that takes
you straight to the source file.

To preview locally:

```bash
cd github-io
bundle install
bundle exec jekyll serve
# http://127.0.0.1:4000/vision_pilot/
```

Pages are Markdown with YAML front matter:

```yaml
---
layout: doc
title: Page title
permalink: /github-io/your-page/
lede: One sentence describing what the page covers.
---
```

Add new pages to the `docs_nav` list in `github-io/_config.yml` so they appear in the sidebar.

## Code of conduct and licence

By participating you agree to the
[Code of Conduct]({{ site.links.repo }}/blob/main/CODE_OF_CONDUCT.md). Contributions are made
under the Apache 2.0 licence.

## Contact

For anything that does not fit a public channel, the Privately Owned Vehicle working group lead is
reachable at the address in the
[onboarding guide]({{ site.links.repo }}/blob/main/ONBOARDING.md).

Stay up to date with Autoware on [autoware.org]({{ site.links.autoware }}),
[LinkedIn]({{ site.links.linkedin }}), [YouTube]({{ site.links.youtube }}) and
[Twitter/X](https://twitter.com/AutowareFdn).
