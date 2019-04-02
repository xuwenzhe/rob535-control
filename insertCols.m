function target = insertCols(target, source, insertPos)
 nEle = size(source, 2);
 tp = target(:, insertPos:end);
 target(:, insertPos:insertPos + nEle - 1) = source;
 target(:, insertPos+nEle:end + nEle) = tp;
 return
