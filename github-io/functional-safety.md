---
layout: doc
title: Functional safety
permalink: /github-io/functional-safety/
lede: Vision Pilot is written to be safety-certifiable. This is what that means, what exists today, and what an integrator is responsible for.
description: Vision Pilot functional safety documentation - safety plan, SEooC scope, software requirements and safety metrics.
---

<div class="note warn" markdown="1">
**Vision Pilot is not a certified system, and is not a finished product.** It is designed to be
productionizable and safety-certifiable, but any deployment on a real vehicle - and every
consequence of that deployment - is the responsibility of the integrator. Read the
[disclaimer]({{ site.links.repo }}/blob/main/DISCLAIMER.md) before going further.
</div>

## The documents

All functional safety material lives in
[`Functional_Safety/`]({{ site.links.repo }}/tree/main/Functional_Safety) in the repository, and is
maintained alongside the code rather than as a separate deliverable.

<div class="grid grid-2">
  <a class="card" href="{{ site.links.repo }}/blob/main/Functional_Safety/SAFETY_PLAN.md">
    <span class="card-tag">Process</span>
    <h3>Safety plan ↗</h3>
    <p>How safety work is organised across the project - activities, artefacts and responsibilities.</p>
  </a>
  <a class="card" href="{{ site.links.repo }}/blob/main/Functional_Safety/SAFETY_ELEMENT_OUT_OF_CONTEXT.md">
    <span class="card-tag">Scope · draft</span>
    <h3>SEooC scope definition ↗</h3>
    <p>The ISO 26262 compliance boundary: what is inside the element, what is assumed of the
       system around it, and what is explicitly out of scope.</p>
  </a>
  <a class="card" href="{{ site.links.repo }}/blob/main/Functional_Safety/SOFTWARE_REQUIREMENTS.md">
    <span class="card-tag">Requirements</span>
    <h3>Software requirements ↗</h3>
    <p>The requirements the implementation is written against, and traced to.</p>
  </a>
  <a class="card" href="{{ site.links.repo }}/blob/main/Functional_Safety/SAFETY_METRICS.md">
    <span class="card-tag">Evidence</span>
    <h3>Safety metrics ↗</h3>
    <p>The measures used to evaluate whether the system behaves safely, and how they are computed.</p>
  </a>
</div>

## Safety Element out of Context

Vision Pilot is developed as a **Safety Element out of Context (SEooC)** under ISO 26262 - a
component designed without a specific vehicle item in mind, shipped together with the assumptions
it makes about the system it will be integrated into.

The SEooC document covers:

| Section | Why it matters to you |
| --- | --- |
| **SEooC scope** | What the element is, and is not |
| **SEooC boundary** | Where Vision Pilot's responsibility ends and yours begins |
| **External interfaces** | What crosses that boundary, in both directions |
| **Assumptions of use** | Conditions that must hold for the safety argument to be valid |
| **Known limitations** | Where the system is known to be weak |
| **Intended ISO 26262 compliance scope** | What is being claimed, and at what level |
| **Future safety roadmap** | What is still to come |

<div class="note" markdown="1">
The **assumptions of use** and **known limitations** sections are the two an integrator must read
in full. An assumption that does not hold in your vehicle invalidates the argument built on it.
</div>

## How safety shapes the architecture

The [architecture]({{ '/github-io/architecture/' | relative_url }}) is built around the safety case
rather than bolted to it:

- **Two independent AI paths.** A perception path producing explicit, inspectable quantities, and
  an end-to-end path. Independence is what makes disagreement detectable.
- **The Safety Guardian sits between AI and actuation.** Nothing reaches the planner without
  passing through fusion. Where the paths disagree, the conservative interpretation is taken.
- **Explicit intermediate quantities.** Object boxes, CIPO distance, ego path and lane state are
  all inspectable, loggable and testable - which is what makes requirements traceable to
  behaviour.
- **A stated sensor specification.** 50–55° FoV, 2 MP, one RGB camera, specified mounting and a
  calibrated homography. These are safety-relevant constraints, not recommendations. See
  [hardware and calibration]({{ '/github-io/hardware/' | relative_url }}).

## Verification you can run today

- **Open-loop replay** against recorded sequences with ground-truth speed logs -
  [getting started]({{ '/github-io/getting-started/#run-on-the-sample-dataset' | relative_url }}).
- **Closed-loop simulation** in CARLA, where the stack's own commands move the vehicle -
  [simulation]({{ '/github-io/simulation/' | relative_url }}).
- **Frame-level logging** to Rerun, so any run can be reconstructed and inspected after the fact -
  [modules]({{ '/github-io/modules/#logging' | relative_url }}).

## On the roadmap

Safety verification and standards compliance against **ISO 26262** (functional safety) and
**ISO 8800** (safety and AI) are tracked on the
[roadmap]({{ '/github-io/releases/#roadmap' | relative_url }}). The SEooC scope definition is currently
a **draft** and will be revised as that work progresses.

Contributions to the safety work are welcome - it is discussed in the weekly
[working group meetings]({{ site.links.discussions }}).
