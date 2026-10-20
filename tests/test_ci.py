"""CI integration: markdown, badge, PR comment create-then-update."""

import io
import json

from cacc import ci

REPORT = {
    "verdict": "FAIL", "counts": {"PASS": 1, "FAIL": 1, "ERROR": 0}, "seeds": [1],
    "under_test": {"controller": "cacc"},
    "cases": [{"name": "hard-brake | nominal", "verdict": "PASS", "checks": []},
              {"name": "hard-brake | spoof", "verdict": "FAIL", "checks": []}],
    "campaign": {
        "maneuvers": [{"key": "hard-brake", "label": "Hard brake"}],
        "conditions": [{"key": "nominal"}, {"key": "spoof"}],
        "grid": [[{"verdict": "PASS", "metric": "min_gap", "worst": 3.2},
                  {"verdict": "FAIL", "metric": "min_gap", "worst": -1.5}]]},
}


def test_markdown_grid_and_badge():
    md = ci.markdown(REPORT)
    assert md.startswith(ci.MARKER) and "**FAIL**" in md
    assert "| Hard brake | ✅ min_gap 3.2 | ❌ min_gap -1.5 |" in md
    svg = ci.badge_svg(REPORT)
    assert "FAIL 1/2" in svg and ci.COLOR["FAIL"] in svg


class FakeGitHub:
    def __init__(self):
        self.comments, self.calls = [], []

    def __call__(self, req):
        body = json.loads(req.data) if req.data else None
        self.calls.append((req.get_method(), req.full_url))
        if req.get_method() == "GET":
            out = self.comments
        elif req.get_method() == "POST":
            self.comments.append({"id": 7, "body": body["body"]})
            out = self.comments[-1]
        else:
            self.comments[0]["body"] = body["body"]
            out = self.comments[0]
        return io.BytesIO(json.dumps(out).encode())


def test_comment_is_created_once_then_edited():
    gh = FakeGitHub()
    md = ci.markdown(REPORT)
    assert ci.post_comment(md, "o/r", 5, "t", opener=gh) == "created"
    assert ci.post_comment(md + "x", "o/r", 5, "t", opener=gh) == "updated"
    assert len(gh.comments) == 1 and gh.comments[0]["body"].endswith("x")
    assert gh.calls[-1] == ("PATCH", "https://api.github.com/repos/o/r/issues/comments/7")


def test_cli_ci_writes_badge_and_summary(tmp_path, monkeypatch):
    from cacc.cli import main

    monkeypatch.delenv("GITHUB_EVENT_PATH", raising=False)
    rep = tmp_path / "report.json"
    rep.write_text(json.dumps(REPORT), encoding="utf-8")
    summary = tmp_path / "sum.md"
    assert main(["ci", str(rep), "--badge", str(tmp_path / "b.svg"),
                 "--summary", str(summary), "--comment"]) == 0
    assert "Hard brake" in summary.read_text(encoding="utf-8")
    assert (tmp_path / "b.svg").is_file()
    assert main(["ci", str(rep), "--gate"]) == 1
