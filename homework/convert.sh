#!/bin/bash

# Converts a jupyter notebook to a PDF

# Convert all SVGs in the given directory to PDFs
find "$1" -name '*.svg' -type f -exec bash -c 'rsvg-convert -f pdf -o "${0%.svg}.pdf" "$0"' {} \;

# Convert notebook to LaTeX and replace \includegraphics{*.svg} with \includegraphics{*.pdf}
jupyter nbconvert "$1/$1.ipynb" --stdout --to latex | sed '/\\includegraphics/{s/\.svg/\.pdf/g}' > "$1/$1.tex"

# Save LaTeX output, rather than piping into pdflatex because it's nice to be able to edit it.

# Attempt to compile the PDF
cd "$1"
latexmk -pdf "$1.tex"
