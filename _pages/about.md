---
layout: about
title: about
permalink: /
subtitle: >
  Research Engineer, <a href="https://www.foundationegi.com/">Foundation EGI</a>

profile:
  align: right
  image: prof_pic_small.jpg
  image_circular: true # crops the image to make it circular
  more_info: arvi.gjoka@nyu.edu

selected_papers: true # includes a list of papers marked as "selected={true}"
social: true # includes social icons at the bottom of the page

announcements:
  enabled: false # includes a list of news items

latest_posts:
  enabled: true
  scrollable: true # adds a vertical scroll bar if there are more than 3 new posts items
  limit: 3 # leave blank to include all the blog posts
---

I am a Research Engineer at [Foundation EGI](https://www.foundationegi.com/), where I work on problems at the intersection of large language models and geometry processing research.

I received my PhD from the Courant Institute, NYU, where I was a member of the [Geometric Computing Lab](https://cims.nyu.edu/gcl/), co-advised by Daniele Panozzo and Denis Zorin. My research asked a simple question: instead of designing soft, flexible objects by trial and error, can a computer design them for us? My approach was to build physics simulations that not only predict how an object will bend, stretch and collide, but can also be run "in reverse" to work out what shape or material will produce a desired behavior.

My work builds on [PolyFEM](https://github.com/polyfem/polyfem), an open-source simulator for deformable objects in contact, which we extended so it can automatically tune an object's shape, material or starting motion to hit a target ([TOG 2024](https://doi.org/10.1145/3657648)). I then used it to design soft robots powered by air. Given the pose you want a robot to strike or the force you want it to apply, the method shapes the hollow chambers inside it to make that happen ([SIGGRAPH 2024](https://doi.org/10.1145/3641519.3657467)). Most recently, I applied the same ideas to stretchable sensors that wrap around an object and detect how it's bending. Here the computer designs the sensor's layout so it can tell different poses apart, with little of the usual tedious calibration ([TOG 2025](https://doi.org/10.1145/3763274)).

I also enjoy building things. I 3D-printed and molded silicone prototypes of these designs to check that the simulations hold up in the real world, which often pointed to new problems worth solving. Earlier in my PhD, I worked on benchmarking machine learning methods for simulating physical systems ([NeurIPS 2021](https://openreview.net/forum?id=pY9MHwmrymR)). Along the way, I interned at Disney Research Zurich and nTop.

Before my PhD, I graduated with a BA in Physics and Computer Science at NYU. I was also a software engineer at Google, where I worked on vision pipelines for long-tailed problems.

<!-- Link to my [resume]({{ site.url }}/assets/pdf/resume.pdf). -->

<!-- <span style="color:#cb4b16">**I am currently looking for industry positions starting in Fall 2025, please reach out if interested!**</span> -->

<!-- Write your biography here. Tell the world about yourself. Link to your favorite [subreddit](http://reddit.com). You can put a picture in, too. The code is already in, just name your picture `prof_pic.jpg` and put it in the `img/` folder.

Put your address / P.O. box / other info right below your picture. You can also disable any of these elements by editing `profile` property of the YAML header of your `_pages/about.md`. Edit `_bibliography/papers.bib` and Jekyll will render your [publications page](/al-folio/publications/) automatically.

Link to your social media connections, too. This theme is set up to use [Font Awesome icons](https://fontawesome.com/) and [Academicons](https://jpswalsh.github.io/academicons/), like the ones below. Add your Facebook, Twitter, LinkedIn, Google Scholar, or just disable all of them. -->
