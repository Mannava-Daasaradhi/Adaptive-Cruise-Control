"""Slide theme + layout primitives for the Control Systems deck.

16:9. EVERY slide is white -- no dark bands, no dark dividers -- because the
deck is projected and printed, and a dark panel is the first thing to fail on
both. Colour carries meaning only through text and thin rules.

Font sizes autofit, but they autofit DOWNWARD ONLY TO A FLOOR (10.5 pt body,
10 pt equations, 9.5 pt tables). Below that the text stops being readable from
the back of a room, so instead of shrinking further the slide is recorded in
`fit_report()` and the build prints it -- the fix for an overfull slide is to
split it, not to set it in 7 pt.
"""

from __future__ import annotations

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.util import Emu, Inches, Pt

# ----------------------------------------------------------------- palette --
INK = RGBColor(0x0F, 0x25, 0x3F)        # headings, dark navy
ACCENT = RGBColor(0x1B, 0x6E, 0xA8)     # primary blue  -- "the paper"
ACCENT2 = RGBColor(0x0E, 0x7C, 0x66)    # teal          -- "our extension"
AMBER = RGBColor(0xB5, 0x6B, 0x00)      # caution / findings
CRIMSON = RGBColor(0xA8, 0x2B, 0x2B)    # problems / instability
TEXT = RGBColor(0x1F, 0x24, 0x28)
MUTED = RGBColor(0x45, 0x50, 0x5B)      # darkened: 0x5B6670 was too light
PANEL = RGBColor(0xF1, 0xF4, 0xF8)
PANEL2 = RGBColor(0xE8, 0xEF, 0xF6)
RULE = RGBColor(0xD3, 0xDB, 0xE3)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)

BODY_FONT = "Calibri"
MONO_FONT = "Consolas"

# ------------------------------------------------------------------ layout --
SLIDE_W = Inches(13.333)
SLIDE_H = Inches(7.5)
MARGIN = Inches(0.55)
CONTENT_TOP = Inches(1.42)
CONTENT_H = Inches(5.45)
CONTENT_W = SLIDE_W - 2 * MARGIN

_STATE = {"n": 0, "section": ""}

# Slides whose autofit bottomed out at its floor with content still to place.
# Raising the font floors converts "silently unreadable" into "reported", which
# is the only way to keep a 24-page deck honest without eyeballing every page.
_OVERFLOW: list = []


def fit_report():
    return list(_OVERFLOW)


def _note_fit(kind, title, size, floor, need, avail):
    if size <= floor + 1e-9 and need > avail + 1e-3:
        _OVERFLOW.append((kind, title, round(size, 2),
                          round(need, 2), round(avail, 2)))


def new_deck() -> Presentation:
    prs = Presentation()
    prs.slide_width = SLIDE_W
    prs.slide_height = SLIDE_H
    _STATE["n"] = 0
    _STATE["section"] = ""
    _OVERFLOW.clear()
    return prs


def set_section(name: str) -> None:
    _STATE["section"] = name


# ------------------------------------------------------------- primitives --
def _blank(prs):
    return prs.slides.add_slide(prs.slide_layouts[6])


def _tb(slide, x, y, w, h, *, anchor=MSO_ANCHOR.TOP):
    box = slide.shapes.add_textbox(x, y, w, h)
    tf = box.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = anchor
    tf.margin_left = tf.margin_right = Emu(0)
    tf.margin_top = tf.margin_bottom = Emu(0)
    return box, tf


# ----------------------------------------------------------- maths markup --
# `_x` / `^x` and `_{...}` / `^{...}` become real sub/superscript runs.
SUPSUB_SCALE = 0.78          # of the surrounding size -- 0.66 was unreadable
                             # once the base font was already near its floor
SUB_BASELINE = "-21000"      # OOXML: fraction of an em below the baseline
SUP_BASELINE = "33000"

_ARG_CHARS = set("abcdefghijklmnopqrstuvwxyz"
                 "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
                 "αβγδεζηθλμνξπρστφχψω")   # H_θ, γ_ρ, ...


def _combining(ch):
    return "\u0300" <= ch <= "\u036f"


def _bare_base_ok(text, i):
    """True when the character before text[i] is a ONE-letter maths base.

    k_v, h_w, 2k_p, e_i  -> yes (base is a single letter)
    run_all, noise_K, min_stable_headway -> no (base is a whole word)
    """
    j = i - 1
    while j >= 0 and _combining(text[j]):
        j -= 1
    if j < 0 or not text[j].isalpha():
        return False
    k = j - 1
    while k >= 0 and _combining(text[k]):
        k -= 1
    return k < 0 or not text[k].isalpha()


def _split_math(text):
    """[(chunk, kind)] with kind in {'', 'sub', 'sup'}."""
    out, buf, i, n = [], [], 0, len(text)
    while i < n:
        ch = text[i]
        if ch in "_^" and i + 1 < n:
            if text[i + 1] == "{":
                j = text.find("}", i + 2)
                arg, end, braced = (text[i + 2:j], j + 1, True) if j != -1 \
                    else (None, None, False)
            else:
                j = i + 1
                while j < n and text[j] in _ARG_CHARS:
                    j += 1
                arg, end, braced = (text[i + 1:j], j, False) if j > i + 1 \
                    else (None, None, False)
            if arg and (braced or _bare_base_ok(text, i)):
                if buf:
                    out.append(("".join(buf), ""))
                    buf = []
                out.append((arg, "sub" if ch == "_" else "sup"))
                i = end
                continue
        buf.append(ch)
        i += 1
    if buf:
        out.append(("".join(buf), ""))
    return out


def _hang(p, mar_l_in, indent_in):
    """Set a hanging indent so WRAPPED lines align under the text, not the bullet."""
    pPr = p._pPr if p._pPr is not None else p._p.get_or_add_pPr()
    pPr.set('marL', str(int(mar_l_in * 914400)))
    pPr.set('indent', str(int(indent_in * 914400)))


def _para(tf, text, *, size, bold=False, color=TEXT, font=BODY_FONT,
          space_before=0, space_after=2, first=False, align=PP_ALIGN.LEFT,
          indent=0, italic=False, hang=None, math=True):
    """One paragraph, split into runs so `_x` / `^x` render as real sub- and
    superscripts. Pass math=False for text that must survive verbatim."""
    p = tf.paragraphs[0] if first else tf.add_paragraph()
    p.alignment = align
    p.space_before = Pt(space_before)
    p.space_after = Pt(space_after)
    if hang is not None:
        _hang(p, hang[0], hang[1])
    parts = _split_math(text) if math else [(text, "")]
    for chunk, kind in parts:
        if not chunk:
            continue
        r = p.add_run()
        r.text = chunk
        f = r.font
        f.size = Pt(size * (SUPSUB_SCALE if kind else 1.0))
        f.bold = bold
        f.italic = italic
        f.color.rgb = color
        f.name = font
        if kind:
            f._rPr.set("baseline",
                       SUB_BASELINE if kind == "sub" else SUP_BASELINE)
    if not parts:                                   # empty string
        p.add_run().text = ""
    return p


def rect(slide, x, y, w, h, fill=PANEL, line=None, line_w=0.75):
    sh = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, x, y, w, h)
    sh.shadow.inherit = False
    if fill is None:
        sh.fill.background()
    else:
        sh.fill.solid()
        sh.fill.fore_color.rgb = fill
    if line is None:
        sh.line.fill.background()
    else:
        sh.line.color.rgb = line
        sh.line.width = Pt(line_w)
    sh.text_frame.word_wrap = True
    return sh


def _footer(slide, title):
    _STATE["n"] += 1
    n = _STATE["n"]
    line = slide.shapes.add_shape(
        MSO_SHAPE.RECTANGLE, MARGIN, Inches(6.97), CONTENT_W, Pt(0.75))
    line.shadow.inherit = False
    line.fill.solid()
    line.fill.fore_color.rgb = RULE
    line.line.fill.background()

    _, tf = _tb(slide, MARGIN, Inches(7.04), Inches(9.5), Inches(0.3))
    _para(tf, _STATE["section"], size=10.5, color=MUTED, first=True)

    _, tf = _tb(slide, SLIDE_W - MARGIN - Inches(3.0), Inches(7.04),
                Inches(3.0), Inches(0.3))
    _para(tf, f"CACC under noisy V2V  |  {n}", size=10.5, color=MUTED,
          first=True, align=PP_ALIGN.RIGHT)
    return n


def header(slide, title, subtitle=None, *, accent=ACCENT):
    bar = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(0), Inches(0),
                                 Pt(9), SLIDE_H)
    bar.shadow.inherit = False
    bar.fill.solid()
    bar.fill.fore_color.rgb = accent
    bar.line.fill.background()

    # shrink a long title until it fits ONE line: the subtitle sits at a fixed
    # y, so a wrapped title would collide with it.
    tsize = 30.0
    while tsize > 21.0 and _wrapped(title, tsize * 1.04, _emu_in(CONTENT_W)) > 1:
        tsize -= 0.5
    _, tf = _tb(slide, MARGIN, Inches(0.42), CONTENT_W, Inches(0.62))
    _para(tf, title, size=tsize, bold=True, color=INK, first=True)
    if subtitle:
        _, tf2 = _tb(slide, MARGIN, Inches(1.02), CONTENT_W, Inches(0.34))
        _para(tf2, subtitle, size=14, color=accent, first=True, italic=True)



# ------------------------------------------------------------- autofit ----
BOTTOM = Inches(6.86)          # hard floor: the footer rule sits at 6.97"

# Autofit will not go below these. A slide that needs less is overfull and is
# reported by fit_report() so it can be SPLIT rather than set in unreadable type.
BODY_FLOOR = 10.5
EQ_FLOOR = 11.0
TABLE_FLOOR = 9.5

NOTE_PT = 12.0


def _note_h(note, width_in=None):
    """Height a footer note actually needs, in inches (0 when there is none)."""
    if not note:
        return 0.0
    w = width_in if width_in is not None else _emu_in(CONTENT_W)
    return _wrapped(note, NOTE_PT, w) * NOTE_PT * 1.30 / 72.0 + 0.07


def _wrapped(text, font_pt, width_in, mono=False):
    """Estimated wrapped line count for `text` at `font_pt` in `width_in`."""
    if not text:
        return 1
    cw = (0.550 if mono else 0.482) * font_pt / 72.0
    cpl = max(8, int(width_in / cw))
    return max(1, -(-len(text) // cpl))


def _emu_in(v):
    return v / 914400.0


# ----------------------------------------------------------------- slides --
def title_slide(prs, title, subtitle, meta_lines):
    """White title page: a thin accent spine at the left edge and rules to
    separate the three blocks. The old dark navy band was the only non-white
    area in the deck; structure here comes from rules and weight instead."""
    s = _blank(prs)
    rect(s, Inches(0), Inches(0), Pt(11), SLIDE_H, fill=ACCENT)
    rect(s, MARGIN, Inches(0.92), Inches(1.7), Pt(4), fill=ACCENT2)

    _, tf = _tb(s, MARGIN, Inches(1.25), Inches(12.0), Inches(1.7))
    _para(tf, title, size=40, bold=True, color=INK, first=True)

    _, tf = _tb(s, MARGIN, Inches(3.02), Inches(11.9), Inches(0.75))
    _para(tf, subtitle, size=16, color=ACCENT, first=True)

    rect(s, MARGIN, Inches(3.98), CONTENT_W, Pt(1), fill=RULE)

    _, tf = _tb(s, MARGIN, Inches(4.2), Inches(12.0), Inches(2.5))
    for i, ln in enumerate(meta_lines):
        _para(tf, ln, size=14 if i == 0 else 12.8,
              color=INK if i == 0 else TEXT if i < 3 else MUTED,
              bold=(i == 0), first=(i == 0), space_after=6)
    return s


def team_slide(prs, members, *, kicker="TEAM", title="Project Team",
               subtitle=None, note=None):
    """Unnumbered team page, styled like the title page: white ground, accent
    spine, and one panel card per member laid out two-up.

    members: list of (name, roll_number).  Kept off the footer counter so the
    numbered body of the deck still starts at 1.
    """
    s = _blank(prs)
    rect(s, Inches(0), Inches(0), Pt(11), SLIDE_H, fill=ACCENT)
    rect(s, MARGIN, Inches(0.92), Inches(1.7), Pt(4), fill=ACCENT2)

    _, tf = _tb(s, MARGIN, Inches(1.12), Inches(11.6), Inches(0.34))
    _para(tf, kicker, size=14, bold=True, color=ACCENT2, first=True)

    _, tf = _tb(s, MARGIN, Inches(1.55), Inches(12.0), Inches(0.8))
    _para(tf, title, size=36, bold=True, color=INK, first=True)

    if subtitle:
        _, tf = _tb(s, MARGIN, Inches(2.36), Inches(11.9), Inches(0.5))
        _para(tf, subtitle, size=15, color=ACCENT, first=True)

    rect(s, MARGIN, Inches(2.98), CONTENT_W, Pt(1), fill=RULE)

    gap = Inches(0.4)
    card_w = int((CONTENT_W - gap) / 2)
    card_h = Inches(1.2)
    row_gap = Inches(0.26)
    top = Inches(3.28)

    for i, member in enumerate(members):
        name, roll = member[0], member[1]
        col, row = i % 2, i // 2
        x = MARGIN + col * (card_w + gap)
        y = top + row * (card_h + row_gap)
        rect(s, x, y, card_w, card_h, fill=PANEL, line=RULE)
        rect(s, x, y, Pt(4), card_h, fill=ACCENT2)

        _, tf = _tb(s, x + Inches(0.28), y + Inches(0.22),
                    card_w - Inches(0.5), Inches(0.42))
        _para(tf, name, size=20, bold=True, color=INK, first=True, math=False)

        _, tf = _tb(s, x + Inches(0.28), y + Inches(0.72),
                    card_w - Inches(0.5), Inches(0.32))
        _para(tf, roll, size=13, color=MUTED, font=MONO_FONT, first=True,
              math=False)

    if note:
        rows = (len(members) + 1) // 2
        y = top + rows * (card_h + row_gap) + Inches(0.18)
        _, tf = _tb(s, MARGIN, y, Inches(12.0), Inches(0.6))
        _para(tf, note, size=12.8, color=MUTED, first=True)
    return s


def section_slide(prs, kicker, title, blurb):
    """White divider -- kept white for the same reason as the title page."""
    set_section(title)
    s = _blank(prs)
    rect(s, Inches(0), Inches(0), Pt(11), SLIDE_H, fill=ACCENT)
    rect(s, MARGIN, Inches(2.5), Inches(1.2), Pt(4), fill=ACCENT2)
    _, tf = _tb(s, MARGIN, Inches(1.95), Inches(11.6), Inches(0.4))
    _para(tf, kicker, size=14, bold=True, color=ACCENT2, first=True)
    _, tf = _tb(s, MARGIN, Inches(2.82), Inches(11.6), Inches(1.1))
    _para(tf, title, size=34, bold=True, color=INK, first=True)
    _, tf = _tb(s, MARGIN, Inches(4.1), Inches(10.6), Inches(1.6))
    _para(tf, blurb, size=15, color=TEXT, first=True)
    _STATE["n"] += 1
    return s


def bullets(prs, title, items, *, subtitle=None, accent=ACCENT,
            base=15.0, top=None, left=None, width=None, slide=None,
            note=None, _reserve=0.0):
    """items: list of (level, text, style) or plain strings.

    style in {'', 'b' bold, 'a' accent, 'g' good/teal, 'r' problem,
              'm' mono, 'q' muted, 'w' amber, 'h' sub-heading}

    The font size is chosen by MEASURING the wrapped height of every item and
    shrinking until the block fits above the footer rule. A dense deck's
    failure mode is silent overflow, so it is prevented rather than eyeballed.
    """
    if slide is None:
        s = _blank(prs)
        header(s, title, subtitle, accent=accent)
        _footer(s, title)
    else:
        s = slide
    x = left if left is not None else MARGIN
    y0 = top if top is not None else CONTENT_TOP
    w = width if width is not None else CONTENT_W

    norm = []
    for it in items:
        if isinstance(it, str):
            norm.append((0, it, ""))
        elif len(it) == 2:
            norm.append((it[0], it[1], ""))
        else:
            norm.append(tuple(it))

    nh = _note_h(note)
    limit = BOTTOM - Inches((nh + 0.08 if note else 0) + _reserve)
    avail_in = _emu_in(limit - y0)
    w_in = _emu_in(w)

    def _size_of(lvl, style, sz):
        fs = sz - (0.8 if lvl else 0)
        if style == "q":
            fs -= 0.9
        elif style == "m":
            fs -= 0.9
        elif style == "h":
            fs += 0.8
        return fs

    def height(sz):
        tot = 0.0
        for k, (lvl, text, style) in enumerate(norm):
            fs = _size_of(lvl, style, sz)
            hang_in = 0.17 if lvl == 0 else 0.19 + 0.22 * lvl
            nl = _wrapped(text, fs, max(1.0, w_in - hang_in),
                          mono=(style == "m"))
            tot += nl * fs * 1.30 / 72.0
            tot += (2.5 if lvl == 0 else 1.5) / 72.0
            if style == "h" and k:
                tot += 5 / 72.0
        return tot

    size = base
    while size > BODY_FLOOR and height(size) > avail_in:
        size -= 0.25
    _note_fit("bullets", title, size, BODY_FLOOR, height(size), avail_in)

    _, tf = _tb(s, x, y0, w, limit - y0)
    for i, (lvl, text, style) in enumerate(norm):
        color, bold, font, italic = TEXT, False, BODY_FONT, False
        sz = _size_of(lvl, style, size)
        if style == "b":
            bold = True
            color = INK
        elif style == "a":
            color = accent
            bold = True
        elif style == "g":
            color = ACCENT2
            bold = True
        elif style == "r":
            color = CRIMSON
            bold = True
        elif style == "w":
            color = AMBER
            bold = True
        elif style == "q":
            color = MUTED
            italic = True
        elif style == "m":
            font = MONO_FONT
            color = INK
        elif style == "h":
            bold = True
            color = INK
        bullet, hang = "", (0.0, 0.0)
        if text and style not in ("h", "m") and lvl == 0:
            bullet, hang = "\u25aa  ", (0.17, -0.17)
        elif text and lvl >= 1 and style not in ("m",):
            bullet, hang = "\u2013  ", (0.19 + 0.22 * lvl, -0.19)
        elif lvl >= 1:
            hang = (0.22 * lvl, 0.0)
        _para(tf, bullet + text, size=sz, bold=bold, color=color, font=font,
              italic=italic, first=(i == 0), hang=hang,
              space_before=(5 if style == "h" and i else 0),
              space_after=(2.5 if lvl == 0 else 1.5))
    if note:
        _, tf = _tb(s, MARGIN, BOTTOM - Inches(nh), CONTENT_W, Inches(nh + 0.1))
        _para(tf, note, size=NOTE_PT, color=MUTED, italic=True, first=True)
    return s


def two_col(prs, title, left_head, left_items, right_head, right_items,
            *, subtitle=None, accent=ACCENT, base=13.5, note=None,
            left_color=None, right_color=None):
    s = _blank(prs)
    header(s, title, subtitle, accent=accent)
    _footer(s, title)
    colw = (CONTENT_W - Inches(0.4)) / 2
    lc = left_color or accent
    rc = right_color or ACCENT2
    for x, head, items, col in ((MARGIN, left_head, left_items, lc),
                                (MARGIN + colw + Inches(0.4), right_head,
                                 right_items, rc)):
        rect(s, x, CONTENT_TOP, colw, Inches(0.42), fill=col)
        _, tf = _tb(s, x + Inches(0.12), CONTENT_TOP + Inches(0.07),
                    colw - Inches(0.2), Inches(0.34))
        _para(tf, head, size=13, bold=True, color=WHITE, first=True)
        bullets(prs, title, items, slide=s, top=CONTENT_TOP + Inches(0.58),
                left=x, width=colw, base=base, accent=col,
                _reserve=_note_h(note))
    if note:
        nh = _note_h(note)
        _, tf = _tb(s, MARGIN, BOTTOM - Inches(nh), CONTENT_W, Inches(nh + 0.1))
        _para(tf, note, size=NOTE_PT, color=MUTED, italic=True, first=True)
    return s


def table_slide(prs, title, headers, rows, *, subtitle=None, accent=ACCENT,
                col_w=None, note=None, base=12.5, highlight_rows=()):
    """A grid of rectangles + textboxes (not a real PPT table, so it can be
    styled per cell).

    Row heights are computed from the WRAPPED content of every cell, and the
    font shrinks until the whole grid plus its note fits above the footer --
    fixed row heights silently clipped multi-line cells.
    """
    s = _blank(prs)
    header(s, title, subtitle, accent=accent)
    _footer(s, title)

    ncol = len(headers)
    widths = col_w or [1.0] * ncol
    total = float(sum(widths))
    widths = [int(w * CONTENT_W / total) for w in widths]
    w_in = [_emu_in(w) - 0.14 for w in widths]

    def cell_text(cell):
        return cell if isinstance(cell, str) else cell[0]

    def lines_in(cell, ci, fs):
        n = 0
        for seg in cell_text(cell).split("\n"):
            n += _wrapped(seg, fs, max(0.4, w_in[ci]))
        return n

    limit = BOTTOM - Inches(_note_h(note) + 0.20 if note else 0.05)
    avail_in = _emu_in(limit - CONTENT_TOP)

    def grid(fs):
        hs = [max(1, max(_wrapped(h, fs, max(0.4, w_in[i]))
                         for i, h in enumerate(headers)))]
        for row in rows:
            hs.append(max(1, max(lines_in(c, i, fs - 0.4)
                                 for i, c in enumerate(row))))
        line = fs * 1.30 / 72.0
        return [n * line + 0.115 for n in hs]

    fs = base
    while fs > TABLE_FLOOR and sum(grid(fs)) > avail_in:
        fs -= 0.25
    _note_fit("table", title, fs, TABLE_FLOOR, sum(grid(fs)), avail_in)
    heights = grid(fs)

    y = CONTENT_TOP
    hh = Inches(heights[0])
    x = MARGIN
    for w, htxt in zip(widths, headers):
        rect(s, x, y, w, hh, fill=accent)
        _, tf = _tb(s, x + Inches(0.07), y + Inches(0.045), w - Inches(0.12), hh)
        _para(tf, htxt, size=fs, bold=True, color=WHITE, first=True)
        x += w
    y += hh

    for ri, row in enumerate(rows):
        rh = Inches(heights[ri + 1])
        fill = PANEL if ri % 2 == 0 else WHITE
        if ri in highlight_rows:
            fill = RGBColor(0xFF, 0xF3, 0xDA)
        x = MARGIN
        for ci, (w, cell) in enumerate(zip(widths, row)):
            rect(s, x, y, w, rh, fill=fill, line=RULE, line_w=0.5)
            txt, style = (cell, "") if isinstance(cell, str) else cell
            color, bold, font = TEXT, False, BODY_FONT
            if style == "b":
                bold = True
                color = INK
            elif style == "g":
                color = ACCENT2
                bold = True
            elif style == "r":
                color = CRIMSON
                bold = True
            elif style == "w":
                color = AMBER
                bold = True
            elif style == "m":
                font = MONO_FONT
            elif style == "q":
                color = MUTED
            _, tf = _tb(s, x + Inches(0.07), y + Inches(0.04),
                        w - Inches(0.12), rh)
            for li, seg in enumerate(txt.split("\n")):
                _para(tf, seg, size=fs - 0.4, bold=bold, color=color,
                      font=font, first=(li == 0), space_after=0.5)
            x += w
        y += rh

    if note:
        nh = _note_h(note)
        _, tf = _tb(s, MARGIN, min(y + Inches(0.14), BOTTOM - Inches(nh)),
                    CONTENT_W, Inches(nh + 0.1))
        _para(tf, note, size=NOTE_PT, color=MUTED, italic=True, first=True)
    return s


def eq_slide(prs, title, blocks, *, subtitle=None, accent=ACCENT, note=None,
             base=14.0):
    """blocks: list of (kind, payload).

    kind 'h' heading, 't' text line, 'e' equation block (list of pre-wrapped
    monospace lines), 'n' small italic note under an equation.

    Like `bullets`, the whole stack is measured first and the base size reduced
    until it fits above the footer. Equation slides are the densest in the deck
    and were the ones that silently overflowed.
    """
    s = _blank(prs)
    header(s, title, subtitle, accent=accent)
    _footer(s, title)

    limit = BOTTOM - Inches(_note_h(note) + 0.10 if note else 0)
    avail_in = _emu_in(limit - CONTENT_TOP)
    w_in = _emu_in(CONTENT_W)

    def layout(sz, apply=False):
        y = CONTENT_TOP
        tot = 0.0
        for kind, payload in blocks:
            if kind == "h":
                fs = sz + 0.5
                nl = _wrapped(payload, fs, w_in)
                h = Inches(nl * fs * 1.34 / 72.0 + 0.055)
                if apply:
                    _, tf = _tb(s, MARGIN, y, CONTENT_W, h)
                    _para(tf, payload, size=fs, bold=True, color=INK,
                          first=True)
                y += h
                tot += _emu_in(h)
            elif kind in ("t", "n"):
                fs = sz - (0.7 if kind == "t" else 1.6)
                col = TEXT if kind == "t" else MUTED
                xoff = Inches(0) if kind == "t" else Inches(0.10)
                nl = _wrapped(payload, fs, w_in - _emu_in(xoff))
                h = Inches(nl * fs * 1.32 / 72.0 + 0.045)
                if apply:
                    _, tf = _tb(s, MARGIN + xoff, y, CONTENT_W - xoff, h)
                    _para(tf, payload, size=fs, color=col, first=True,
                          italic=(kind == "n"))
                y += h
                tot += _emu_in(h)
            elif kind == "e":
                fs = sz - 0.5
                lines = payload
                h = Inches(0.05 + len(lines) * (fs * 1.302 / 72.0))
                if apply:
                    rect(s, MARGIN, y, CONTENT_W, h, fill=PANEL2)
                    rect(s, MARGIN, y, Pt(2.5), h, fill=accent)
                    _, tf = _tb(s, MARGIN + Inches(0.18), y + Inches(0.05),
                                CONTENT_W - Inches(0.3), h)
                    for i, ln in enumerate(lines):
                        _para(tf, ln, size=fs, color=INK, font=MONO_FONT,
                              first=(i == 0), space_after=0.5)
                y += h + Inches(0.115)
                tot += _emu_in(h) + 0.115
        return tot

    sz = base
    while sz > EQ_FLOOR and layout(sz) > avail_in:
        sz -= 0.25
    _note_fit("eq", title, sz, EQ_FLOOR, layout(sz), avail_in)
    layout(sz, apply=True)

    if note:
        nh = _note_h(note)
        _, tf = _tb(s, MARGIN, BOTTOM - Inches(nh), CONTENT_W, Inches(nh + 0.1))
        _para(tf, note, size=NOTE_PT, color=MUTED, italic=True, first=True)
    return s


DERIV_FLOOR = 10.0


def derivation_slide(prs, title, steps, *, subtitle=None, goal=None,
                     closer=None, accent=ACCENT, base=14.0, note=None):
    """A derivation told as numbered steps rather than a wall of algebra.

    goal    -- one plain-English sentence: what this page proves and why anyone
               should care. Sits in a tinted strip above the first step.
    steps   -- list of (badge, purpose, math_lines, result):
               badge       "1", "2", ... (or "" for an unnumbered aside)
               purpose     plain English -- what this step DOES
               math_lines  list of monospace lines (may be [] for prose steps)
               result      plain English -- what the algebra just told us, or
                           None. Rendered in teal, prefixed with an arrow.
    closer  -- the conclusion strip at the foot of the page.

    The reason this exists rather than another `eq_slide`: an equation block on
    its own tells a reader WHAT was written, never WHY, and the derivations
    were the pages people bounced off. Purpose and result lines cost vertical
    space, so the whole stack is measured and shrunk exactly as eq_slide does.
    """
    s = _blank(prs)
    header(s, title, subtitle, accent=accent)
    _footer(s, title)

    limit = BOTTOM - Inches(_note_h(note) + 0.10 if note else 0)
    avail_in = _emu_in(limit - CONTENT_TOP)
    w_in = _emu_in(CONTENT_W)

    IND = Inches(0.42)           # text column, clear of the step badge
    ind_w = CONTENT_W - IND
    ind_in = _emu_in(ind_w)

    def strip(y, sz, label, text, tint, tick, apply):
        """GOAL / conclusion band. Returns its height in EMU."""
        lh = sz - 3.0
        bh = sz - 0.2
        h = Inches(0.055 + lh * 1.30 / 72.0
                   + _wrapped(text, bh, _emu_in(CONTENT_W) - 0.42) * bh * 1.32 / 72.0
                   + 0.07)
        if apply:
            rect(s, MARGIN, y, CONTENT_W, h, fill=tint)
            rect(s, MARGIN, y, Pt(3), h, fill=tick)
            _, tf = _tb(s, MARGIN + Inches(0.20), y + Inches(0.055),
                        CONTENT_W - Inches(0.35), h)
            _para(tf, label, size=lh, bold=True, color=tick, first=True,
                  space_after=1.5, math=False)
            _para(tf, text, size=bh, color=INK, space_after=0)
        return h

    def layout(sz, apply=False):
        y = CONTENT_TOP
        tot = 0.0

        if goal:
            h = strip(y, sz, "GOAL", goal, PANEL, ACCENT2, apply)
            y += h + Inches(0.13)
            tot += _emu_in(h) + 0.13

        for badge, purpose, math_lines, result in steps:
            ps = sz + 0.2
            nl = _wrapped(purpose, ps, ind_in)
            ph = Inches(nl * ps * 1.34 / 72.0 + 0.05)
            if apply:
                if badge:
                    b = rect(s, MARGIN, y + Inches(0.015),
                             Inches(0.30), Inches(0.30), fill=accent)
                    b.text_frame.margin_left = b.text_frame.margin_right = Emu(0)
                    b.text_frame.margin_top = b.text_frame.margin_bottom = Emu(0)
                    b.text_frame.vertical_anchor = MSO_ANCHOR.MIDDLE
                    _para(b.text_frame, badge, size=sz - 2.0, bold=True,
                          color=WHITE, first=True, align=PP_ALIGN.CENTER,
                          space_after=0, math=False)
                _, tf = _tb(s, MARGIN + IND, y, ind_w, ph)
                _para(tf, purpose, size=ps, bold=True, color=INK, first=True)
            y += ph
            tot += _emu_in(ph)

            if math_lines:
                fs = sz - 0.6
                mh = Inches(0.05 + len(math_lines) * (fs * 1.302 / 72.0))
                if apply:
                    rect(s, MARGIN + IND, y, ind_w, mh, fill=PANEL2)
                    rect(s, MARGIN + IND, y, Pt(2.5), mh, fill=accent)
                    _, tf = _tb(s, MARGIN + IND + Inches(0.16),
                                y + Inches(0.045), ind_w - Inches(0.26), mh)
                    for i, ln in enumerate(math_lines):
                        _para(tf, ln, size=fs, color=INK, font=MONO_FONT,
                              first=(i == 0), space_after=0.5)
                y += mh + Inches(0.035)
                tot += _emu_in(mh) + 0.035

            if result:
                rs = sz - 1.4
                txt = "⟹   " + result
                nl = _wrapped(txt, rs, ind_in - 0.16)
                rh = Inches(nl * rs * 1.32 / 72.0 + 0.05)
                if apply:
                    _, tf = _tb(s, MARGIN + IND + Inches(0.16), y,
                                ind_w - Inches(0.16), rh)
                    _para(tf, txt, size=rs, color=ACCENT2, bold=True,
                          first=True)
                y += rh
                tot += _emu_in(rh)

            y += Inches(0.135)
            tot += 0.135

        if closer:
            h = strip(y, sz, "WHAT THIS BUYS US", closer, PANEL2, accent, apply)
            tot += _emu_in(h)
        return tot

    sz = base
    while sz > DERIV_FLOOR and layout(sz) > avail_in:
        sz -= 0.25
    _note_fit("deriv", title, sz, DERIV_FLOOR, layout(sz), avail_in)
    layout(sz, apply=True)

    if note:
        nh = _note_h(note)
        _, tf = _tb(s, MARGIN, BOTTOM - Inches(nh), CONTENT_W, Inches(nh + 0.1))
        _para(tf, note, size=NOTE_PT, color=MUTED, italic=True, first=True)
    return s


MATH_FLOOR = 9.5
EQ_SCALE = 1.62          # rendered-equation height per point of body size


def math_slide(prs, title, items, *, subtitle=None, accent=ACCENT, note=None,
               base=13.0, goal=None, closer=None):
    """Typeset equations with a plain-English gloss under each one.

    items -- list of (badge, heading, tex_lines, gloss):
        badge      "1".."n", or "" for an unnumbered block
        heading    what this equation IS, in caps-ish plain words
        tex_lines  list of mathtext strings (no $) -- rendered by eqmath to
                   real fraction bars, subscripts and Greek
        gloss      one sentence: what it MEANS. Rendered as "IN WORDS: ...".

    This replaces the monospace ASCII blocks the deck used to carry. An
    equation written as `(a s^2 + b s + c) / (d s^3 + ...)` on one line is
    technically complete and practically unreadable; a reader who cannot parse
    it cannot present it. The gloss is not decoration -- it is the line the
    presenter actually says out loud.
    """
    from . import eqmath

    s = _blank(prs)
    header(s, title, subtitle, accent=accent)
    _footer(s, title)

    limit = BOTTOM - Inches(_note_h(note) + 0.10 if note else 0)
    avail_in = _emu_in(limit - CONTENT_TOP)

    IND = Inches(0.42)
    ind_w = CONTENT_W - IND
    ind_in = _emu_in(ind_w)

    nat = {}                                   # tex -> (w_in, h_in) at RENDER_PT
    for _b, _h, tex_lines, _g in items:
        for tex in tex_lines:
            if tex not in nat:
                nat[tex] = eqmath.measure(tex)

    def strip(y, sz, label, text, tint, tick, apply):
        lh = sz - 3.0
        bh = sz - 0.2
        h = Inches(0.055 + lh * 1.30 / 72.0
                   + _wrapped(text, bh, _emu_in(CONTENT_W) - 0.42) * bh * 1.32 / 72.0
                   + 0.07)
        if apply:
            rect(s, MARGIN, y, CONTENT_W, h, fill=tint)
            rect(s, MARGIN, y, Pt(3), h, fill=tick)
            _, tf = _tb(s, MARGIN + Inches(0.20), y + Inches(0.055),
                        CONTENT_W - Inches(0.35), h)
            _para(tf, label, size=lh, bold=True, color=tick, first=True,
                  space_after=1.5, math=False)
            _para(tf, text, size=bh, color=INK, space_after=0)
        return h

    def layout(sz, apply=False):
        y = CONTENT_TOP
        tot = 0.0

        if goal:
            h = strip(y, sz, "GOAL", goal, PANEL, ACCENT2, apply)
            y += h + Inches(0.12)
            tot += _emu_in(h) + 0.12

        scale = (sz * EQ_SCALE) / eqmath.RENDER_PT
        max_w = ind_in - 0.56

        for badge, heading, tex_lines, gloss in items:
            hs = sz + 0.2
            hh = Inches(_wrapped(heading, hs, ind_in) * hs * 1.34 / 72.0 + 0.04)
            if apply:
                if badge:
                    b = rect(s, MARGIN, y + Inches(0.015),
                             Inches(0.30), Inches(0.30), fill=accent)
                    b.text_frame.margin_left = b.text_frame.margin_right = Emu(0)
                    b.text_frame.margin_top = b.text_frame.margin_bottom = Emu(0)
                    b.text_frame.vertical_anchor = MSO_ANCHOR.MIDDLE
                    _para(b.text_frame, badge, size=sz - 2.0, bold=True,
                          color=WHITE, first=True, align=PP_ALIGN.CENTER,
                          space_after=0, math=False)
                _, tf = _tb(s, MARGIN + IND, y, ind_w, hh)
                _para(tf, heading, size=hs, bold=True, color=INK, first=True)
            y += hh
            tot += _emu_in(hh)

            if tex_lines:
                rows = []
                for tex in tex_lines:
                    w0, h0 = nat[tex]
                    w, h = w0 * scale, h0 * scale
                    if w > max_w:                  # keep aspect, fit the column
                        h *= max_w / w
                        w = max_w
                    rows.append((tex, w, h))
                gap = 0.11
                panel_in = sum(r[2] for r in rows) + gap * (len(rows) - 1) + 0.16
                ph = Inches(panel_in)
                if apply:
                    rect(s, MARGIN + IND, y, ind_w, ph, fill=PANEL2)
                    rect(s, MARGIN + IND, y, Pt(2.5), ph, fill=accent)
                    yy = _emu_in(y) + 0.08
                    for tex, w, h in rows:
                        path, _, _ = eqmath.render(tex)
                        s.shapes.add_picture(
                            str(path), MARGIN + IND + Inches(0.28), Inches(yy),
                            width=Inches(w), height=Inches(h))
                        yy += h + gap
                y += ph + Inches(0.045)
                tot += panel_in + 0.045

            if gloss:
                gs = sz - 0.9
                txt = "IN WORDS:   " + gloss
                gh = Inches(_wrapped(txt, gs, ind_in - 0.16) * gs * 1.32 / 72.0
                            + 0.05)
                if apply:
                    _, tf = _tb(s, MARGIN + IND + Inches(0.16), y,
                                ind_w - Inches(0.16), gh)
                    _para(tf, txt, size=gs, color=ACCENT2, bold=True,
                          first=True)
                y += gh
                tot += _emu_in(gh)

            y += Inches(0.15)
            tot += 0.15

        if closer:
            h = strip(y, sz, "WHAT THIS BUYS US", closer, PANEL2, accent, apply)
            tot += _emu_in(h)
        return tot

    sz = base
    while sz > MATH_FLOOR and layout(sz) > avail_in:
        sz -= 0.25
    _note_fit("math", title, sz, MATH_FLOOR, layout(sz), avail_in)
    layout(sz, apply=True)

    if note:
        nh = _note_h(note)
        _, tf = _tb(s, MARGIN, BOTTOM - Inches(nh), CONTENT_W, Inches(nh + 0.1))
        _para(tf, note, size=NOTE_PT, color=MUTED, italic=True, first=True)
    return s


def figure_slide(prs, title, img_path, caption_items, *, subtitle=None,
                 accent=ACCENT, img_frac=0.46, base=13.5, note=None):
    """One figure on the left, a bullet block on the right, optional note under.

    The figure is capped so it cannot run into the note; the bullets autofit in
    the remaining column exactly as they do on a full-width slide.
    """
    s = _blank(prs)
    header(s, title, subtitle, accent=accent)
    _footer(s, title)
    img_w = int(CONTENT_W * img_frac)
    cap_h = CONTENT_H - (Inches(0.55) if note else Inches(0))
    try:
        pic = s.shapes.add_picture(str(img_path), MARGIN, CONTENT_TOP,
                                   width=img_w)
        if pic.height > cap_h:
            scale = cap_h / pic.height
            pic.height = int(pic.height * scale)
            pic.width = int(pic.width * scale)
        used = pic.width
    except Exception:                                     # missing figure
        rect(s, MARGIN, CONTENT_TOP, img_w, Inches(3.2), fill=PANEL)
        used = img_w
    tx = MARGIN + used + Inches(0.35)
    bullets(prs, title, caption_items, slide=s, top=CONTENT_TOP, left=tx,
            width=CONTENT_W - used - Inches(0.35), base=base, accent=accent,
            note=note)
    return s


def figures_slide(prs, title, panels, *, subtitle=None, accent=ACCENT,
                  note=None, foot_items=None, base=13.0, img_frac=0.66):
    """panels: list of (img_path, caption) laid out in ONE row.

    Reproduction evidence is comparative -- the attenuating case only means
    something next to the amplifying one -- so the figures have to share a page
    at the same scale rather than appear on consecutive slides.

    `foot_items` is an optional bullet block under the row; the row is sized
    first and the bullets take whatever vertical space is left.
    """
    s = _blank(prs)
    header(s, title, subtitle, accent=accent)
    _footer(s, title)

    n = len(panels)
    gap = Inches(0.3)
    colw = int((CONTENT_W - gap * (n - 1)) / n)
    limit = BOTTOM - Inches(_note_h(note) + 0.08 if note else 0.0)
    fig_h = int((limit - CONTENT_TOP) * img_frac)

    bottoms = []
    x = MARGIN
    for path, _cap in panels:
        try:
            pic = s.shapes.add_picture(str(path), x, CONTENT_TOP, width=colw)
            if pic.height > fig_h:
                sc = fig_h / pic.height
                pic.height = int(pic.height * sc)
                pic.width = int(pic.width * sc)
                pic.left = x + int((colw - pic.width) / 2)
            bottoms.append(CONTENT_TOP + pic.height)
        except Exception:                                  # missing figure
            rect(s, x, CONTENT_TOP, colw, fig_h, fill=PANEL)
            bottoms.append(CONTENT_TOP + fig_h)
        x += colw + gap

    cy = max(bottoms) + Inches(0.09)
    cap_lines = max(_wrapped(c, 11.5, _emu_in(colw)) for _p, c in panels)
    cap_h = Inches(cap_lines * 11.5 * 1.3 / 72.0 + 0.06)
    x = MARGIN
    for _path, cap in panels:
        _, tf = _tb(s, x, cy, colw, cap_h)
        _para(tf, cap, size=11.5, color=MUTED, italic=True, first=True)
        x += colw + gap

    if foot_items:
        bullets(prs, title, foot_items, slide=s, top=cy + cap_h + Inches(0.10),
                base=base, accent=accent)
    if note:
        nh = _note_h(note)
        _, tf = _tb(s, MARGIN, BOTTOM - Inches(nh), CONTENT_W, Inches(nh + 0.1))
        _para(tf, note, size=NOTE_PT, color=MUTED, italic=True, first=True)
    return s


def big_stat_slide(prs, title, stats, *, subtitle=None, accent=ACCENT,
                   footer_items=None):
    """stats: list of (value, label, sublabel, color)."""
    s = _blank(prs)
    header(s, title, subtitle, accent=accent)
    _footer(s, title)
    n = len(stats)
    gap = Inches(0.28)
    w = int((CONTENT_W - gap * (n - 1)) / n)
    x = MARGIN
    for value, label, sub, col in stats:
        rect(s, x, CONTENT_TOP, w, Inches(2.05), fill=PANEL)
        rect(s, x, CONTENT_TOP, w, Pt(3), fill=col)
        _, tf = _tb(s, x + Inches(0.14), CONTENT_TOP + Inches(0.24),
                    w - Inches(0.28), Inches(0.8))
        _para(tf, value, size=32, bold=True, color=col, first=True)
        _, tf = _tb(s, x + Inches(0.14), CONTENT_TOP + Inches(1.0),
                    w - Inches(0.28), Inches(0.4))
        _para(tf, label, size=14, bold=True, color=INK, first=True)
        _, tf = _tb(s, x + Inches(0.14), CONTENT_TOP + Inches(1.36),
                    w - Inches(0.28), Inches(0.7))
        _para(tf, sub, size=12, color=MUTED, first=True)
        x += w + gap
    if footer_items:
        bullets(prs, title, footer_items, slide=s,
                top=CONTENT_TOP + Inches(2.35), base=13.5, accent=accent)
    return s
