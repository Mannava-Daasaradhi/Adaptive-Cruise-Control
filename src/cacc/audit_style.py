"""Stylesheet of the audit report (light and dark themes, print)."""

CSS = """
:root { color-scheme: light; --ink:#0b0b0b; --muted:#52514e; --line:#e4e3df;
  --surface:#fcfcfb; --panel:#f4f3f0; --ok-bg:#eaf6ea;
  --s1:#2a78d6; --s2:#eb6834; --s3:#1baf7a;
  --good-ink:#067a06; --critical-ink:#b02e2e; --warning-ink:#8a5d00;
  --good:#0ca30c; --warning:#fab219; --critical:#d03b3b; }
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) { color-scheme: dark; --ink:#f2f1ec; --muted:#c3c2b7; --line:#3a3936;
  --surface:#1a1a19; --panel:#262624; --ok-bg:#1f3a1f;
  --s1:#3987e5; --s2:#d95926; --s3:#199e70;
  --good-ink:#5fd35f; --critical-ink:#f08a8a; --warning-ink:#f5c040; }
}
:root[data-theme="dark"] { color-scheme: dark; --ink:#f2f1ec; --muted:#c3c2b7; --line:#3a3936;
  --surface:#1a1a19; --panel:#262624; --ok-bg:#1f3a1f;
  --s1:#3987e5; --s2:#d95926; --s3:#199e70;
  --good-ink:#5fd35f; --critical-ink:#f08a8a; --warning-ink:#f5c040; }
* { box-sizing: border-box; }
body { margin: 0; background: var(--surface); color: var(--ink);
  font: 14px/1.5 -apple-system, "Segoe UI", Roboto, Helvetica, Arial,
  sans-serif; }
main { max-width: 860px; margin: 0 auto; padding: 32px 16px 64px; }
h1 { font-size: 26px; margin: 0 0 4px; }
h2 { font-size: 20px; margin: 0 0 12px; display: flex; flex-wrap: wrap;
  gap: 8px; align-items: baseline; }
h3 { font-size: 15px; margin: 22px 0 6px; }
.sub, .note, .meta { color: var(--muted); }
.sub { font-size: 14px; font-weight: 400; }
.note { font-size: 13px; margin: 0 0 8px; }
table { border-collapse: collapse; width: 100%; font-size: 13px;
  margin: 6px 0 10px; }
th, td { border-bottom: 1px solid var(--line); padding: 5px 8px;
  text-align: left; vertical-align: top; }
thead th { color: var(--muted); font-weight: 600; }
.num { font-variant-numeric: tabular-nums; white-space: nowrap; }
.grid td { font-variant-numeric: tabular-nums; white-space: nowrap; }
.grid td.ok { background: var(--ok-bg); }
.grid td.na { color: var(--muted); }
.badge { font-size: 12px; font-weight: 600; padding: 2px 8px;
  border-radius: 999px; border: 1.5px solid; white-space: nowrap; }
.badge.good { border-color: var(--good); color: var(--good-ink); }
.badge.critical { border-color: var(--critical); color: var(--critical-ink); }
.badge.warning { border-color: var(--warning); color: var(--warning-ink); }
.kpis { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr));
  gap: 8px; margin: 8px 0 12px; }
.kpis div { background: var(--panel); border-radius: 8px; padding: 8px 10px;
  min-width: 0; }
.kpis span { display: block; font-size: 12px; color: var(--muted); }
.kpis b { font-size: 18px; font-variant-numeric: tabular-nums; }
.rec { background: var(--panel); border-left: 3px solid var(--ink);
  padding: 8px 12px; border-radius: 4px; }
figure { margin: 6px 0 12px; overflow-x: auto; }
figure svg { max-width: 100%; height: auto; }
figure svg text { fill: var(--ink); }  /* chart text without its own fill */
section.vehicle { border-top: 2px solid var(--ink); margin-top: 36px;
  padding-top: 16px; }
.scroll { overflow-x: auto; }
code { font-size: 12px; background: var(--panel); padding: 1px 4px;
  border-radius: 3px; overflow-wrap: anywhere; }
@media (max-width: 640px) {
  .kpis { grid-template-columns: repeat(2, minmax(0, 1fr)); }
  figure svg { min-width: 560px; max-width: none; }  /* scroll, stay legible */
}
@media print {
  :root:not([data-theme="print"]) { color-scheme: light; --ink:#0b0b0b; --muted:#52514e; --line:#e4e3df;
  --surface:#fcfcfb; --panel:#f4f3f0; --ok-bg:#eaf6ea;
  --s1:#2a78d6; --s2:#eb6834; --s3:#1baf7a;
  --good-ink:#067a06; --critical-ink:#b02e2e; --warning-ink:#8a5d00; }
  body { background: white; }
  main { padding: 0; max-width: none; }
  section.vehicle { break-before: page; }
  figure, table { break-inside: avoid; }
}
"""
