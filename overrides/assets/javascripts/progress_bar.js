document.addEventListener("DOMContentLoaded", function () {
  document.querySelectorAll(".roadmap-progress-track").forEach(function (track) {
    var fill = track.querySelector(".roadmap-progress-fill");
    var now = new Date();
    var year = now.getFullYear();
    var month = now.getMonth(); // 0-indexed, May = 4

    // Determine the current May-to-May cycle
    var startYear = month >= 4 ? year : year - 1;
    var endYear = startYear + 1;
    var start = new Date(startYear, 4, 1);
    var end = new Date(endYear, 4, 1);

    // Set date labels
    var datesEl = track.closest(".roadmap-progress").querySelector(".roadmap-progress-dates");
    if (datesEl) {
      var spans = datesEl.querySelectorAll("span");
      if (spans.length >= 2) {
        spans[0].textContent = "May " + startYear;
        spans[1].textContent = "May " + endYear;
      }
    }

    // If we're in May, show full bar
    if (month === 4) {
      fill.style.width = "100%";
      return;
    }

    var total = end - start;
    var elapsed = now - start;
    var pct = Math.max(0, Math.min(100, (elapsed / total) * 100));
    fill.style.width = Math.round(pct) + "%";
  });
});
