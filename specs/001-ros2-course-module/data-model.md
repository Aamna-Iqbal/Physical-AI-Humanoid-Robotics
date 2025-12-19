# Data Model for ROS 2 Course Module

This feature is a documentation site, so the data model is based on the file system structure used by Docusaurus.

## Entities

### Course Module
*   **Represents**: The entire ROS 2 course.
*   **Implementation**: A Docusaurus site instance. The content for the modules will be inside the `docs` directory.
*   **Attributes**:
    *   `title`: The title of the course.

### Chapter
*   **Represents**: A distinct section of the course.
*   **Implementation**: A Markdown file (`.md`) within a subdirectory of the `docs` directory.
*   **Attributes**:
    *   `title`: The title of the chapter, specified in the Markdown frontmatter.
    *   `content`: The main text of the chapter in Markdown format.
    *   `sidebar_position`: A number in the frontmatter to control the order in the sidebar.

### Code Example
*   **Represents**: A small, runnable piece of code used for illustration.
*   **Implementation**: A fenced code block within a Chapter's Markdown file.
*   **Attributes**:
    *   `language`: The programming language of the code (e.g., `python`, `xml`).
    *   `code`: The source code of the example.

## Structure

The planned directory structure for the Docusaurus content will be:

```
/docs
|-- module-1-ros2
|   |-- 1-ros-2-fundamentals.md
|   |-- 2-python-agents-with-ros-2.md
|   |-- 3-humanoid-modeling-with-urdf.md
|-- module-2-placeholer.md
|-- module-3-placeholer.md
...
```
This structure allows for clear organization of modules and chapters, and Docusaurus will automatically generate sidebar navigation based on this structure.
