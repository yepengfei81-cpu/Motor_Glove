# 组会 PPT 生成脚本

    cd tools/deck && npm install && NODE_PATH=$PWD/node_modules node build_deck.js      # 输出 docs/lab_meeting_2026-10-15.pptx

- 版式、配色、每页内容都在 build_deck.js 里；图片来自 docs/figs/ppt/、docs/figs/ 和 docs/figs/papers/（论文配图，arXiv HTML 版下载）。
- 把 P8 的 [VIDEO] 和 P9 的 [PLOT] 占位框换成实物视频、`plot_log.py` 的图：在脚本里把 placeholderBox(...) 改成 img(s, 路径, x, y, w, h)。
- 质检（可选）：pptx skill 的 scripts/office/validate.py 和 soffice.py --convert-to pdf。
