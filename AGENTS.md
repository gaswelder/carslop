# Project conventions

- Use structs with named fields instead of tuples for data whose members have
  distinct meanings.

- Avoid import aliases. When a member name is ambiguous, import only its namespace and use namespaced references.

- Declare imports with `use` instead of writing inline `crate::` paths.

- Avoid re-exports. Import items directly from the module that defines them.

- Do not combine a namespace and its members in `{self, ...}` imports. Import only the namespace and use `::` references for its members.

- Keep items private unless production code outside their module needs them. Tests are not a reason to expose implementation details: put such tests in the owning module or remove them.

- Don't use pub(crate) when simply pub works.

- Keep tests in separate files, following the existing module layout.

- Use `impl` blocks only for trait implementations. Define other operations as module-level functions.
