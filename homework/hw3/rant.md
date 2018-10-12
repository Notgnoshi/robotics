# Rant

So I use Jupyter notebooks for mathematical programming because they rock. They're especially handy because you can convert them to a *large* number of other formats, like HTML, ReStructured Text, PDF, LaTeX, etc. However, when I get my homework to a happy place about four hours ago, I went to convert it to LaTeX (so I could edit the styles before compiling) Jupyter crashed.

Now, before you ask, I've used the same process for HW1 and HW2. I've even used SVG images in both (which, as you'll learn in just a bit, are quite literally the devil incarnate when LaTeX is concerned.)

```shell
jupyter nbconvert --to latex hw3.ipynb
jupyter nbconvert --to latex hw3.ipynb --debug
jupyter nbconvert --to latex hw3.ipynb --debug --ExtractOutputPreprocessor.enabled=True
jupyter nbconvert --to latex hw3.ipynb --debug --ExtractOutputPreprocessor.enabled=False
```

Crap, it crashes, stating that it cannot convert a `bytes` object to JSON... After digging a little deeper in the Output Preprocessor, it looks like it's failing to recognize that SVG images are indeed images. Weird. I'm doing nothing I've never done before, but I remembered that I had updated Jupyter a few days ago, so I go check to see if the update broke things.

```shell
cd ..
jupyter nbconvert --to latex hw2/hw2.ipynb
jupyter nbconvert --to latex hw1/hw1.ipynb
```

Both worked perfectly. So clearly conversion to LaTeX is broken. But there are other things I can convert to, so I tried ReStructured Text.

```shell
jupyter nbconvert --to rst hw3/hw3.ipynb
```

Awesome, that worked. Maybe I can use Pandoc to convert from ReStructured Text back to LaTeX? But that'll require converting the SVGs to PDFs. No problem, that's something I've done plenty of times before. Often enough I almost remembered the `find` incantation required to recursively convert SVGs to PDFs. Almost.

```shell
cd hw3_files/
find . -name '*.svg' -type f -exec bash -c 'convert "$0" "${0%.svg}.pdf"' {} \;
```

Well shit. It works on SVGs created by my last homeworks, but not this one... So naturually, that's not a problem with the SVG files, it's a problem with the SVG converter.

```shell
sudo apt install librsvg2-bin
find . -name '*.svg' -type f -exec bash -c 'rsvg-convert -f pdf -o "${0%.svg}.pdf" "$0"' {} \;
pandoc hw3.rst -o hw3.tex
```

Well shoot me in the back of the head. It worked. Let's replace the SVGs images in the LaTeX source with PDFs and try to compile! What could go wrong!?

```shell
sed -i '/\\includegraphics/{s/\.svg/\.pdf/g}' hw3.tex
latexmk -pdf -shell-escape hw3.tex
man pandoc
```

Well, it turns out that's a bust. Pandoc actually sucks at syntax highlighting (I didn't try very hard), and left out half the document!!

The next thing I notice is that `nbconvert` apparently has a dependency on a GTK module?! Now, why in the flipping hell would a document conversion utility depend on a module from a GUI toolkit? Beats me.

```shell
jupyter nbconvert --stdout --to latex hw3/hw3.ipynb
sudo apt install libcanberra-gtk-module
jupyter nbconvert --stdout --to latex hw3/hw3.ipynb
```

It didn't help, but it fixed the warning messages, so that's nice. The next thing to futilely try is the `nbconvert` help page. It turns out there's two of them, and neither are helpful, contrary to their names.

```shell
jupyter nbconvert --help
jupyter nbconvert --help-all
```

So I settled on a solution. Sort of.

I can export as ReStructured Text with the SVG outputs. I can export as LaTeX with PNG outputs. So all I should need to do is convert the exported SVG images to PDFs, do a batch rename (newer versions of Nautilus have a fantastic batch renaming utility, by the way, just select a bunch of files and press F2.), and then replace all `*.png` in the LaTeX source with `*.pdf`.

At this point I should have known better than to ask what could go wrong.

```shell
# Run all cells with SVG turned on
jupyter nbconvert --to rst hw3.ipynb
# Run all cells with PNGs turned on
jupyter nbconvert --to latex hw3.ipynb
find hw3-rst/ -name '*.svg' -type f -exec bash -c 'rsvg-convert -f pdf -o "${0%.svg}.pdf" "$0"' {} \;
cp *.pdf hw3-tex
# There's a hand made SVG to convert
sed -i '/\\includegraphics/{s/\.svg/\.pdf/g}' hw3.tex
# Also convert all the automagically included PNGs
sed -i '/\\includegraphics/{s/\.png/\.pdf/g}' hw3.tex
```

But exporting to RST and LaTeX apparently saves the SVG and PNG images with different filenames! Oh well, it's only 10 files to rename by hand.

But when I attempt to compile, `latexmk` tells me the LaTeX source code is erroneous. Apparently `nbconvert` is smart enough to syntax highlight Python source code using raw TeX primitives, but somehow isn't aware you're not supposed to put an `align*` environment inside of math mode?!

So I fix that and try again.

```shell
cd hw3-tex
vim hw3.tex
latexmk -pdf hw3.tex
```

What do you think happened next? If you guessed a successful compilation, you'd be horribly horribly wrong. It turns out, `nbconvert` doesn't use `\includegraphics` when including a PNG, but does for every other image format? So my awesome use of `sed` a little while ago was entirely unfounded.

```shell
sed -i 's/\.png/\.pdf/g' hw3.tex
latexmk -pdf hw3.tex
```

Boom. Halle-frickin-lujah. It compiled. What's more, all the images are in their correct place, and are high quality vector graphics. Except one.

One of my original troubleshooting steps was to comment out things in the markdown cells of the Jupyter notebook and do a rudimentary bisective search to try to figure out what exactly was going wrong. Well, I forgot to uncomment one of the images. So I had to repeat the whole process again.

Export as RST, export as LaTeX. Convert SVGs to PDFs. Use `sed` to replace `.png` and `.svg` (both, because one of the images was created by hand and saved as SVG) with `.pdf`. Fix the `align*` in math mode. Batch rename the PDF files to match the PNG filenames by hand again. Recompile.

Then I noticed that the generated LaTeX didn't look like the template I disabled while I was troubleshooting.

---

After going through this entire process, I found a way to get Jupyter to save the matplotlib plots as PDFs after 2 minutes of Duck-Duck-Going, which might have saved me from all of this. It's not optimal, because it uses `PDF.js` to render the PDFs in an iframe in the web client, but I think it might actually be able to generate the LaTeX without causing. I think I'll wait until right before the next homework is due to find out though.
