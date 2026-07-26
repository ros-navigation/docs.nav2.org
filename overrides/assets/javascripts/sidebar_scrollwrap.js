function monitorAndUpdateHeight(gapValue, elementSelector) {
  const element = document.querySelector(elementSelector);

  if (!element) return;

  const styleTag = document.createElement('style');
  document.head.appendChild(styleTag);

  const updateHeight = () => {
    const currentHeight = element.style.height;
    if (!currentHeight) return;

    const heightValue = parseFloat(currentHeight);
    const newHeight = heightValue + gapValue;
    styleTag.textContent = `${elementSelector} { height: ${newHeight}px !important; }`;
  };

  const config = { attributes: true, attributeFilter: ['style'] };
  const observer = new MutationObserver(updateHeight);
  observer.observe(element, config);

  updateHeight();
}

document.addEventListener('DOMContentLoaded', () => {
  monitorAndUpdateHeight(72, '.md-sidebar.md-sidebar--primary .md-sidebar__scrollwrap');
});
