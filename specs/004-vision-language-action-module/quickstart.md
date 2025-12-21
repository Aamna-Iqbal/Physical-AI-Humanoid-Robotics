# Quickstart: Adding New Course Content

This guide explains how to add new chapters to the course.

## 1. Create the Markdown File

-   Navigate to the appropriate module directory under `docs-site/docs/`. For example, for Module 4, the directory is `docs-site/docs/module-4-vla-course/`.
-   Create a new `.md` file with a descriptive name (e.g., `4-new-topic.md`).

## 2. Add Content to the File

-   Add standard markdown content to the file.
-   Each file should start with a level 1 heading (`#`) for the chapter title.

## 3. Update the Sidebar

-   Open `docs-site/sidebars.js`.
-   Find the `category` object for the relevant module.
-   Add the path to your new file in the `items` array. The path should be relative to the `docs-site/docs` directory and should not include the `.md` extension.

    ```javascript
    // example for adding a file to module 4
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-vla-course/1-voice-to-action',
        'module-4-vla-course/2-llm-based-planning',
        'module-4-vla-course/3-autonomous-humanoid-capstone',
        'module-4-vla-course/4-new-topic', // <-- new file added here
      ],
    },
    ```

## 4. Verify the Changes

-   Run the Docusaurus development server (`npm start` from the `docs-site` directory) to preview the changes and ensure the new chapter appears correctly in the sidebar.
