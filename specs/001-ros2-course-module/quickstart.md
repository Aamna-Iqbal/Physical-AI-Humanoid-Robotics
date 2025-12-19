# Quickstart for ROS 2 Course Module Development

This guide provides instructions on how to set up the development environment for the Docusaurus-based documentation site.

## Prerequisites

*   [Node.js](https://nodejs.org/) (version 18.x or later)
*   [npm](https://www.npmjs.com/) (usually comes with Node.js)

## Setup

1.  **Initialize the Docusaurus site:**
    The first step will be to create a new Docusaurus site. We will use the `classic` template.

    ```bash
    npx create-docusaurus@latest my-website classic
    ```
    *(Note: `my-website` will be the name of the directory for the site, we might want to name it `docs-site` or something similar)*

2.  **Install dependencies:**
    Navigate into the newly created site directory and install the dependencies.

    ```bash
    cd my-website
    npm install
    ```

## Running the development server

To start the local development server and view the site:

```bash
npm start
```

This will start a local development server and open up a browser window. Most changes are reflected live without having to restart the server.

## Building the static site

To generate a static build of the site for production:

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.
