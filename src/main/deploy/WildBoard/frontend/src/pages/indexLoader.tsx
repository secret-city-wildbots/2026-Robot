import { h, render } from "[DEPLOY]/Wildboard/frontend/src/node_modules/preact";
import Page from "[HOME]/frontend/src/pages/index.tsx";

render(
    <Page />,
  document.getElementsByTagName("main")[0]!
);

