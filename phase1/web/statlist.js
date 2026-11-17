// A label/value list (<dl class="stats">) that stays still while its values change: the rows are built once
// per set of labels and only the changed values are rewritten, and a row never shrinks back after a value
// needed more lines (cleared when the panel width changes), so the panel does not jump as numbers tick.
export function statList(dl) {
  let labels = "";
  let dds = [];
  let shown = [];
  let tall = [];
  const forget = () => {
    tall = dds.map(() => 0);
    dds.forEach((dd) => (dd.style.minHeight = ""));
  };
  let width = 0;
  new ResizeObserver(([e]) => {
    const w = Math.round(e.contentRect.width);
    if (w !== width) (width = w), forget();
  }).observe(dl);
  return {
    // rows: [label, value html, tooltip?]
    set(rows) {
      const key = rows.map((r) => r[0]).join("\n");
      if (key !== labels) {
        labels = key;
        dl.replaceChildren();
        dds = rows.map(([label, , tip]) => {
          const dt = document.createElement("dt");
          dt.textContent = label;
          if (tip) dt.title = tip;
          const dd = document.createElement("dd");
          dl.append(dt, dd);
          return dd;
        });
        shown = rows.map(() => null);
        forget();
      }
      rows.forEach(([, v], k) => {
        if (shown[k] === v) return;
        shown[k] = v;
        dds[k].innerHTML = v;
      });
      dds.forEach((dd, k) => {
        const h = dd.offsetHeight;
        if (h > tall[k]) {
          tall[k] = h;
          dd.style.minHeight = `${h}px`;
        }
      });
    },
  };
}

// Give a button whose label toggles the width of its longest label, so the row of buttons does not reflow.
export function steadyWidth(btn, labels) {
  const now = btn.textContent;
  let w = 0;
  for (const l of labels) {
    btn.textContent = l;
    w = Math.max(w, btn.getBoundingClientRect().width);
  }
  btn.textContent = now;
  if (w > 0) btn.style.minWidth = `${Math.ceil(w)}px`;
}
