# Build the handout: make (latexmk -pdf). TikZ figures in figures/, no external graphics.
LATEXMK := latexmk

polygon_map.pdf: polygon_map.tex figures/hall.tikz figures/pipeline.tikz
	$(LATEXMK) -pdf -interaction=nonstopmode polygon_map.tex

.PHONY: log clean

log:
	@grep -nE "^!|Overfull \\\\hbox \(([1-9][0-9]|[0-9]+[0-9])pt|Undefined control|LaTeX Warning: (Reference|Citation)" polygon_map.log || echo "log clean"

clean:
	-$(LATEXMK) -c polygon_map.tex >/dev/null 2>&1
