# MISRA C++ 2023 analysis (CodeQL)

The [`codeql-misra-cpp`](../workflows/codeql-misra-cpp.yaml) workflow checks VisionPilot's C++ code against [MISRA C++ 2023](https://misra.org.uk/), using the queries from [CodeQL Coding Standards](https://github.com/github/codeql-coding-standards).

VisionPilot is developed as an ISO 26262 Safety Element out of Context (see [`Functional_Safety/`](../../Functional_Safety/)). ISO 26262 Part 6 expects code to follow a coding guideline such as MISRA C++, and this workflow shows how far the code is from that guideline.

## What it does

1. **Sets up the build environment.** It uses [`setup-vision-pilot-deps`](../actions/setup-vision-pilot-deps/action.yaml) on Ubuntu 24.04 with ROS 2 Jazzy.
2. **Downloads the tools.** It downloads the CodeQL CLI and the Coding Standards query packs, checks them against their published checksums, and caches them for later runs.
3. **Configures CMake.** It uses `GPU=OFF`, `ENABLE_ROS2_INTERFACE=ON` and `-lcppad_lib`. Dependencies fetched by CMake (Eigen) are put outside the repository, so their code is not reported as VisionPilot findings. Configure is retried up to 3 times, because gitlab.com sometimes rejects the Eigen download.
4. **Builds the code under CodeQL.** CodeQL records a clean build of the `VisionPilot` target and creates a database from it.
5. **Analyzes the database.** It runs the MISRA C++ 2023 suite in [`misra-cpp-2023.qls`](misra-cpp-2023.qls).
6. **Reports the results:**
   - uploads the SARIF file to GitHub Code Scanning, under the category `misra-cpp-2023`
   - uploads the SARIF file as the `codeql-misra-cpp-2023` artifact
   - writes a table of findings per rule to the job summary.

## When it runs

- **Weekly:** every Monday at 03:00 JST (Sunday 18:00 UTC), on the default branch.
- **Manually:** from the Actions tab, with **Run workflow**.

It does not run on pull requests.

## Report-only

**Findings never fail the job.** The job fails only if the analysis itself breaks, for example when the build or a download fails.

MISRA has three rule categories:

- **Mandatory:** no exceptions are allowed.
- **Required:** an exception (deviation) is allowed if it is justified and approved.
- **Advisory:** a recommendation.

So failing on every Required finding would be stricter than MISRA itself. The plan for gating is:

1. Fail on Mandatory findings. There are none today.
2. Fail on Required findings that have no approved deviation, inside the scope the Safety Plan defines.
3. Fail pull requests only on findings they add, using Code Scanning's comparison with the base branch.

## Where to find the results

- **Security → Code scanning**, filtered by tool `CodeQL` and category `misra-cpp-2023`. Each alert links to the line of code and to the rule's help text.
- **The workflow run page:** the job summary has the per-rule table, and the SARIF file is attached as an artifact.

## Query suite

[`misra-cpp-2023.qls`](misra-cpp-2023.qls) is the default MISRA C++ 2023 suite, with one change:

- The Rule 8.7.1 query `cpp/misra/pointer-arithmetic-forms-an-invalid-pointer` is excluded. In a trial it ran for more than 35 million iterations (about 70 minutes) without finishing. The other Rule 8.7.1 query (`pointer-argument-to-cstring-function-is-invalid`) still runs.
- Until the excluded query can run, pointer arithmetic is reviewed by hand. To retry the query on a newer release, delete the last `exclude` block in the suite file.

## Updating the versions

The versions are set in the workflow's `env` block:

| Variable                   | Meaning                                    |
| -------------------------- | ------------------------------------------ |
| `CODING_STANDARDS_VERSION` | Release tag of CodeQL Coding Standards     |
| `CODEQL_CLI_VERSION`       | CodeQL CLI version that this release needs |

Update them together. Each Coding Standards release supports exactly one CLI version, listed in `supported_codeql_configs.json` at that release's tag. Using the release's own file matters, because the file on `main` may already name the CLI for the next release. The cache key includes both versions, so a change downloads the new tools on the next run.

## Running it locally

```bash
# CodeQL bundle and packs for the versions in the workflow
codeql database create /tmp/codeql-db --language=cpp --source-root=. \
  --command="cmake --build VisionPilot/build --target VisionPilot -j$(nproc)"
codeql database analyze /tmp/codeql-db .github/codeql/misra-cpp-2023.qls \
  --additional-packs=<dir with the extracted misra-cpp-coding-standards pack> \
  --format=sarifv2.1.0 --output=misra-cpp-2023.sarif
```

Before running these commands:

- Configure `VisionPilot/build` the same way as the workflow, with `-DFETCHCONTENT_BASE_DIR` pointing outside the repository.
- Start from a clean build directory, because CodeQL can only analyze files that are compiled while it is watching.

## Limitations

- **Not compliance evidence.** The Coding Standards ISO 26262 tool qualification covers only the validated compilers (gcc 8.4.0, clang 10.0.0, qcc 8.3.0) and use as distributed. Running in GitHub Actions and uploading to Code Scanning is described as compatible with the qualification, but not certified. This workflow uses gcc 13.
- **Only built code is analyzed.** Code that the default configuration does not compile is not analyzed, for example `ENABLE_OCCUPANCY` code, `tests/` and `Simulation/`.
- **Default suite only.** Queries tagged `audit` or `default-disabled` are not run.
