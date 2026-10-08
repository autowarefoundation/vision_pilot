---
layout: doc
title: Releases
permalink: /github-io/releases/
lede: Tagged versions of Vision Pilot, what changed in each, and what is planned next.
description: Vision Pilot release history and roadmap, from v0.5 to the current release.
---

Releases are tagged in the [repository]({{ site.links.repo }}/tags) and published on the
[releases page]({{ site.links.releases }}), where source archives, Debian packages and model
weights are attached.

<div class="note tip" markdown="1">
Current release: **v{{ site.version }}**. Install it from source, as a `.deb`, or via Docker -
see [getting started]({{ '/github-io/getting-started/' | relative_url }}).
</div>

## History

<div class="timeline">
  <div class="tl-item current">
    <div class="tl-head"><span class="v">v1.2</span><span class="pill latest">Latest</span><span class="d">10 August 2026</span></div>
    <ul>
      <li><strong>Rerun logging</strong> - stream per-frame data straight into an <code>.rrd</code> recording (<code>rrd_on</code>, <code>rrd_log</code>), openable with <code>rerun &lt;file&gt;</code>.</li>
      <li><strong>Occupancy view colours</strong> - refined palette for the bird's-eye panel.</li>
      <li>Documentation: Safety Element out of Context (SEooC) scope definition, in draft.</li>
    </ul>
  </div>

  <div class="tl-item">
    <div class="tl-head"><span class="v">v1.1</span><span class="d">27 July 2026</span></div>
    <ul>
      <li><strong>Occupancy BEV window</strong> - optional heuristic 3D / bird's-eye panel beside the HUD, built with <code>-DENABLE_OCCUPANCY=ON</code>, with orbit / pan / zoom controls.</li>
      <li><strong>Camera calibration guide</strong> and the ground-checkerboard homography script.</li>
      <li>Camera selection and mounting guidance for real-vehicle installs.</li>
    </ul>
  </div>

  <div class="tl-item">
    <div class="tl-head"><span class="v">v1.0</span><span class="pill">Milestone</span><span class="d">6 July 2026</span></div>
    <ul>
      <li>First complete L2 feature set - ACC, FCW, AEB, LKAS, LDW, ISA and single-lane highway autopilot - on the hybrid end-to-end architecture.</li>
      <li>Debian packaging via <code>cpack -G DEB</code>, with GPU and CPU variants.</li>
      <li>Docker images for GPU/CPU and with/without ROS 2.</li>
      <li>CARLA 0.9.16 closed-loop support through the ROS 2 bridge.</li>
      <li>WebRTC visualisation streaming for headless and remote monitoring.</li>
    </ul>
  </div>

  <div class="tl-item">
    <div class="tl-head"><span class="v">v0.9</span><span class="d">2 March 2026</span></div>
    <ul>
      <li>Pre-release consolidation ahead of the 1.0 feature freeze.</li>
    </ul>
  </div>

  <div class="tl-item">
    <div class="tl-head"><span class="v">v0.5</span><span class="d">20 January 2026</span></div>
    <ul>
      <li>First public tagged release of the Vision Pilot stack.</li>
    </ul>
  </div>
</div>

For the exact commit range behind any tag, use the
[GitHub compare view]({{ site.links.repo }}/compare) - for example
[`v1.1...v1.2`]({{ site.links.repo }}/compare/v1.1...v1.2).

## Roadmap

<div class="grid grid-2">
  <div class="card">
    <span class="card-tag">Sensing</span>
    <h3>Camera + RADAR fusion</h3>
    <p>Support for fusion between the front-facing camera and automotive RADAR, for robustness in
       conditions where vision alone degrades.</p>
  </div>
  <div class="card">
    <span class="card-tag">Sensing</span>
    <h3>8 MP, 120° FoV</h3>
    <p>Support for higher-resolution, wider-field cameras alongside the current 2 MP, 50–55°
       baseline.</p>
  </div>
  <div class="card">
    <span class="card-tag">Safety</span>
    <h3>Standards compliance</h3>
    <p>Safety verification and compliance work towards ISO 26262 and ISO 8800. See the
       <a href="{{ '/github-io/functional-safety/' | relative_url }}">functional safety documentation</a>.</p>
  </div>
  <div class="card">
    <span class="card-tag">Simulation</span>
    <h3>SODA.Sim</h3>
    <p>Integration alongside the existing CARLA support, joining the ROS 2 and experimental Zenoh
       transports.</p>
  </div>
</div>

## Versioning and compatibility

- **Config files** are read at startup and may gain keys between releases. Diff your customised
  `vision_pilot.conf` against the shipped template after upgrading.
- **Model weights** are versioned with the release. Mixing weights from one release with the
  runtime of another is not supported.
- **`H.yaml`** is specific to your camera and mounting, not to a release - carry yours forward.

## Staying informed

- Watch the [repository]({{ site.links.repo }}) for release notifications.
- Working group meeting minutes and recordings are posted to
  [GitHub Discussions]({{ site.links.discussions }}).
- Announcements also go out on [Discord]({{ site.links.discord }}) and
  [LinkedIn]({{ site.links.linkedin }}).
