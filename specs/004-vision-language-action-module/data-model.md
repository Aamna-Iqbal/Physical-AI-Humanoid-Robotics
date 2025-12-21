# Data Model: Course Content

The data for this feature consists of the course content itself, which is authored as markdown files within the Docusaurus project.

## Entities

### CourseModule

Represents a top-level module in the course.

-   **Attributes**:
    -   `title` (string): The display name of the module (e.g., "Module 4: Vision-Language-Action (VLA)").
    -   `directory` (string): The subdirectory within `docs-site/docs` that holds the module's content.

### Chapter

Represents a single chapter or document within a Course Module.

-   **Attributes**:
    -   `title` (string): The title of the chapter, defined within the markdown file.
    -   `content` (string): The body of the chapter, written in Markdown.
    -   `path` (string): The file path to the markdown file relative to the `docs-site/docs` directory.

## Relationships

-   A `CourseModule` contains one or more `Chapter`s.
-   This relationship is managed by the directory structure and the `docs-site/sidebars.js` configuration file.

## State Transitions

-   Not applicable for this data model, as the content is static.
