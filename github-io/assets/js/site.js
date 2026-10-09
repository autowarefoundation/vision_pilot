/* Vision Pilot docs - small, dependency-free page behaviour. */
(function () {
  'use strict';

  /* Mobile nav ============================================================== */
  var toggle = document.querySelector('.nav-toggle');
  var nav = document.querySelector('.topnav');
  if (toggle && nav) {
    toggle.addEventListener('click', function () {
      var open = nav.classList.toggle('open');
      toggle.setAttribute('aria-expanded', String(open));
    });
  }

  /* Quick-start tabs ======================================================= */
  document.querySelectorAll('.tabs').forEach(function (tabs) {
    var buttons = tabs.querySelectorAll('.tablist button');
    buttons.forEach(function (btn) {
      btn.addEventListener('click', function () {
        buttons.forEach(function (b) {
          var selected = b === btn;
          b.setAttribute('aria-selected', String(selected));
          var panel = tabs.querySelector('#' + b.getAttribute('aria-controls'));
          if (panel) panel.hidden = !selected;
        });
      });
    });
  });

  /* Click-to-load video embeds ============================================
     Nothing third-party is requested until the visitor actually asks for it. */
  document.querySelectorAll('.vframe[data-embed]').forEach(function (frame) {
    frame.addEventListener('click', function () {
      if (frame.dataset.loaded) return;
      frame.dataset.loaded = '1';
      var iframe = document.createElement('iframe');
      iframe.src = frame.dataset.embed;
      iframe.allow = 'autoplay; fullscreen';
      iframe.allowFullscreen = true;
      iframe.title = frame.dataset.title || 'Vision Pilot demo video';
      frame.innerHTML = '';
      frame.appendChild(iframe);
    });
  });

  /* Reveal on scroll ======================================================= */
  var targets = document.querySelectorAll('.reveal');
  if (!('IntersectionObserver' in window)) {
    targets.forEach(function (el) { el.classList.add('in'); });
  } else {
    var io = new IntersectionObserver(function (entries) {
      entries.forEach(function (entry) {
        if (entry.isIntersecting) {
          entry.target.classList.add('in');
          io.unobserve(entry.target);
        }
      });
    }, { rootMargin: '0px 0px -8% 0px', threshold: 0.06 });
    targets.forEach(function (el) { io.observe(el); });
  }

  /* Copy button on every code block ======================================== */
  document.querySelectorAll('pre').forEach(function (pre) {
    if (!navigator.clipboard) return;
    pre.style.position = 'relative';
    var btn = document.createElement('button');
    btn.type = 'button';
    btn.textContent = 'Copy';
    btn.className = 'copy-btn';
    btn.setAttribute('aria-label', 'Copy code to clipboard');
    btn.addEventListener('click', function () {
      navigator.clipboard.writeText(pre.innerText.replace(/\bCopy\b\s*$/, '').trim()).then(function () {
        btn.textContent = 'Copied';
        setTimeout(function () { btn.textContent = 'Copy'; }, 1600);
      });
    });
    pre.appendChild(btn);
  });
})();
